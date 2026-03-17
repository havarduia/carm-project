#!/usr/bin/env python3

import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from xarm.wrapper import XArmAPI

from movement import move as MoveHelper
from yolo_model import YoloSnapshotNode


class PickAndPlaceUI:
    def __init__(self):
        rclpy.init()
        self.detector_node = YoloSnapshotNode()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.detector_node)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.arm = None
        self.mover = None
        self.speed = 50
        self.offset_x = 0.010  # meters
        self.offset_z = -0.020  # meters

        self.root = tk.Tk()
        self.root.title('CARM Component Detection + Move UI')
        self.root.geometry('520x420')

        self.status_var = tk.StringVar(value='Ready. Connect arm and run detection.')
        self.detected_var = tk.StringVar(value='No detection yet')

        self.x_var = tk.StringVar(value='0.0')
        self.y_var = tk.StringVar(value='0.0')
        self.z_var = tk.StringVar(value='0.0')

        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self.close)

    def _build_ui(self):
        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text='Component pick workflow', font=('Arial', 14, 'bold')).pack(anchor=tk.W)
        ttk.Label(
            frame,
            text='1) Detect component  2) Review coordinates  3) Confirm move',
            foreground='gray',
        ).pack(anchor=tk.W, pady=(0, 10))

        button_bar = ttk.Frame(frame)
        button_bar.pack(fill=tk.X, pady=6)

        ttk.Button(button_bar, text='Connect Arm', command=self.connect_arm).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_bar, text='Detect Component', command=self.detect_component).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_bar, text='Use Last Detection', command=self.load_last_detection).pack(side=tk.LEFT, padx=4)

        coord_frame = ttk.LabelFrame(frame, text='Target coordinate in link_base (meters)', padding=10)
        coord_frame.pack(fill=tk.X, pady=10)

        for row, (label, var) in enumerate([('X', self.x_var), ('Y', self.y_var), ('Z', self.z_var)]):
            ttk.Label(coord_frame, text=label, width=4).grid(row=row, column=0, padx=4, pady=4)
            ttk.Entry(coord_frame, textvariable=var, width=18).grid(row=row, column=1, padx=4, pady=4, sticky='w')

        ttk.Label(
            coord_frame,
            text='You can edit these values before movement.',
            foreground='gray',
        ).grid(row=3, column=0, columnspan=2, sticky='w', pady=(8, 0))

        action_bar = ttk.Frame(frame)
        action_bar.pack(fill=tk.X, pady=8)

        ttk.Button(action_bar, text='Move To Target', command=self.move_to_target).pack(side=tk.LEFT, padx=4)
        ttk.Button(action_bar, text='Home', command=self.go_home).pack(side=tk.LEFT, padx=4)

        ttk.Separator(frame).pack(fill=tk.X, pady=8)

        ttk.Label(frame, text='Detection:').pack(anchor=tk.W)
        ttk.Label(frame, textvariable=self.detected_var, wraplength=490).pack(anchor=tk.W, pady=(0, 8))

        ttk.Label(frame, text='Status:').pack(anchor=tk.W)
        ttk.Label(frame, textvariable=self.status_var, wraplength=490).pack(anchor=tk.W)

    def connect_arm(self):
        if self.arm is not None:
            self.status_var.set('Arm already connected.')
            return
        try:
            self.arm = XArmAPI('192.168.1.225')
            time.sleep(0.5)
            self.arm.set_tcp_maxacc(1000)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            self.mover = MoveHelper(speed=self.speed, arm=self.arm)
            self.status_var.set('Arm connected and ready.')
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f'Failed to connect arm: {exc}')

    def detect_component(self):
        result = self.detector_node.detect_latest_frame()
        if result is None:
            self.status_var.set('Detection failed. Make sure camera/depth/TF are available.')
            return
        self.detected_var.set(
            f"{result.label} ({result.confidence:.2f}) at pixel ({result.pixel_u}, {result.pixel_v}) -> "
            f"robot ({result.robot_x:.3f}, {result.robot_y:.3f}, {result.robot_z:.3f})"
        )
        self.x_var.set(f'{result.robot_x:.3f}')
        self.y_var.set(f'{result.robot_y:.3f}')
        self.z_var.set(f'{result.robot_z:.3f}')
        self.status_var.set('Detection complete. Review/edit coordinates, then press Move To Target.')

    def load_last_detection(self):
        result = self.detector_node.last_result
        if result is None:
            self.status_var.set('No previous detection available.')
            return
        self.x_var.set(f'{result.robot_x:.3f}')
        self.y_var.set(f'{result.robot_y:.3f}')
        self.z_var.set(f'{result.robot_z:.3f}')
        self.status_var.set('Loaded last detection into editable coordinate fields.')

    def move_to_target(self):
        if self.arm is None or self.mover is None:
            self.status_var.set('Connect arm before moving.')
            return

        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            z = float(self.z_var.get())
        except ValueError:
            self.status_var.set('Coordinates must be valid numbers.')
            return

        confirmed = messagebox.askyesno(
            'Confirm movement',
            f'Move robot to\nX={x:.3f}, Y={y:.3f}, Z={z:.3f} (m)\nwith offsets applied?',
        )
        if not confirmed:
            self.status_var.set('Movement cancelled by user.')
            return

        def _run_move():
            self.status_var.set('Moving robot...')
            try:
                px, py, pz = 180, 0, 0
                target_x_mm = (x + self.offset_x) * 1000.0
                target_y_mm = y * 1000.0
                target_z_mm = (z + self.offset_z) * 1000.0

                self.mover.home()
                self.arm.set_position(
                    *[target_x_mm, target_y_mm, target_z_mm, px, py, pz],
                    speed=self.speed,
                    wait=True,
                )
                self.arm.set_gripper_position(350, wait=True)
                self.mover.home()
                self.status_var.set('Movement complete.')
            except Exception as exc:  # noqa: BLE001
                self.status_var.set(f'Movement failed: {exc}')

        threading.Thread(target=_run_move, daemon=True).start()

    def go_home(self):
        if self.mover is None:
            self.status_var.set('Connect arm before homing.')
            return

        def _home():
            try:
                self.mover.home()
                self.status_var.set('Moved to home pose.')
            except Exception as exc:  # noqa: BLE001
                self.status_var.set(f'Failed to home: {exc}')

        threading.Thread(target=_home, daemon=True).start()

    def close(self):
        try:
            if self.arm is not None:
                self.arm.disconnect()
        except Exception:  # noqa: BLE001
            pass

        self.executor.shutdown()
        self.detector_node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    app = PickAndPlaceUI()
    app.run()
