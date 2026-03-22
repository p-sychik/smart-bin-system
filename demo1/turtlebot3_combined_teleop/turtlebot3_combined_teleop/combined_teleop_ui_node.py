import tkinter as tk
from tkinter import ttk

import rclpy

from .combined_teleop_node import CombinedTeleopNode, USAGE_MSG


class CombinedTeleopUI:
    def __init__(self, node):
        self.node = node
        self._closing = False

        self.root = tk.Tk()
        self.root.title('TurtleBot3 Combined Teleop UI')
        self.root.minsize(860, 680)
        self.root.configure(padx=14, pady=14)

        self.motion_var = tk.StringVar()
        self.servo_var = tk.StringVar()
        self.recording_var = tk.StringVar()
        self.marker_var = tk.StringVar()
        self.record_count_var = tk.StringVar()

        self._build_layout()
        self._update_status()

        self.root.bind('<KeyPress>', self._on_keypress)
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)
        self.root.after(50, self._tick)

    def _build_layout(self):
        title = ttk.Label(
            self.root,
            text='Combined TurtleBot3 Teleop',
            font=('TkDefaultFont', 16, 'bold'),
        )
        title.pack(anchor='w')

        subtitle = ttk.Label(
            self.root,
            text='Use buttons or keyboard shortcuts (focus this window).',
        )
        subtitle.pack(anchor='w', pady=(0, 8))

        content = ttk.Frame(self.root)
        content.pack(fill='both', expand=True)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)

        controls = ttk.LabelFrame(content, text='Controls')
        controls.grid(row=0, column=0, sticky='nsew', padx=(0, 8), pady=(0, 8))
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)

        usage = ttk.LabelFrame(content, text='Keymap')
        usage.grid(row=0, column=1, sticky='nsew', pady=(0, 8))
        usage.columnconfigure(0, weight=1)

        status = ttk.LabelFrame(content, text='Status')
        status.grid(row=1, column=0, columnspan=2, sticky='nsew')
        status.columnconfigure(1, weight=1)

        self._add_section_label(controls, 'Movement', row=0)
        self._add_button(controls, 'Forward (W)', 'w', row=1, col=0)
        self._add_button(controls, 'Backward (S)', 's', row=1, col=1)
        self._add_button(controls, 'Turn Left (A)', 'a', row=2, col=0)
        self._add_button(controls, 'Turn Right (D)', 'd', row=2, col=1)
        self._add_button(controls, 'Stop + Cancel Seek (X)', 'x',
                         row=3, col=0, colspan=2)

        self._add_section_label(controls, 'Servos', row=4)
        self._add_button(controls, 'Unhook (C)', 'c', row=5, col=0)
        self._add_button(controls, 'Hook (V)', 'v', row=5, col=1)

        self._add_section_label(controls, 'Recording', row=6)
        self._add_button(controls, 'Start Recording (O)', 'o', row=7, col=0)
        self._add_button(controls, 'Stop Recording (P)', 'p', row=7, col=1)
        self._add_button(controls, 'Replay (Space)', ' ', row=8, col=0)
        self._add_button(controls, 'Run Test Sequence (T)', 't', row=8, col=1)

        self._add_section_label(controls, 'Marker Seek', row=9)
        self._add_button(controls, 'Start Seek (F)', 'f', row=10, col=0)
        self._add_button(controls, 'Quit (Q)', 'q', row=10, col=1)

        usage_text = tk.Text(
            usage,
            height=24,
            wrap='word',
            relief='flat',
            borderwidth=0,
            highlightthickness=0,
        )
        usage_text.insert('1.0', USAGE_MSG.strip())
        usage_text.configure(state='disabled')
        usage_text.pack(fill='both', expand=True, padx=8, pady=6)

        ttk.Label(status, text='Motion:').grid(
            row=0, column=0, sticky='w', padx=8, pady=4)
        ttk.Label(status, textvariable=self.motion_var).grid(
            row=0, column=1, sticky='w', pady=4)

        ttk.Label(status, text='Servos:').grid(
            row=1, column=0, sticky='w', padx=8, pady=4)
        ttk.Label(status, textvariable=self.servo_var).grid(
            row=1, column=1, sticky='w', pady=4)

        ttk.Label(status, text='Recording:').grid(
            row=2, column=0, sticky='w', padx=8, pady=4)
        ttk.Label(status, textvariable=self.recording_var).grid(
            row=2, column=1, sticky='w', pady=4)

        ttk.Label(status, text='Recorded Actions:').grid(
            row=3, column=0, sticky='w', padx=8, pady=4)
        ttk.Label(status, textvariable=self.record_count_var).grid(
            row=3, column=1, sticky='w', pady=4)

        ttk.Label(status, text='Marker Seek:').grid(
            row=4, column=0, sticky='w', padx=8, pady=4)
        ttk.Label(status, textvariable=self.marker_var).grid(
            row=4, column=1, sticky='w', pady=4)

    def _add_section_label(self, parent, title, row):
        label = ttk.Label(parent, text=title, font=('TkDefaultFont', 11, 'bold'))
        label.grid(row=row, column=0, columnspan=2, sticky='w',
                   pady=(10 if row else 0, 4))

    def _add_button(self, parent, text, key, row, col, colspan=1):
        button = ttk.Button(parent, text=text, command=lambda: self._dispatch_key(key))
        button.grid(row=row, column=col, columnspan=colspan,
                    sticky='ew', padx=4, pady=4)

    def _on_keypress(self, event):
        if event.keysym == 'space':
            key = ' '
        elif event.char:
            key = event.char
        else:
            return

        allowed = {'w', 'a', 's', 'd', 'x', 'c', 'v', 'o', 'p', 'f', 't', 'q'}
        if key.lower() in allowed or key == ' ':
            self._dispatch_key(key)

    def _dispatch_key(self, key):
        keep_running = self.node.handle_key(key)
        self._update_status()
        if keep_running is False:
            self.shutdown()

    def _update_status(self):
        self.motion_var.set(
            f'linear_x={self.node.linear_x:.2f} m/s, '
            f'angular_z={self.node.angular_z:.2f} rad/s'
        )
        self.servo_var.set(
            f'left={self.node.servo_left:.2f}, right={self.node.servo_right:.2f}'
        )
        self.recording_var.set('ON' if self.node.recording else 'OFF')
        self.record_count_var.set(str(len(self.node.recorded_actions)))
        if self.node.marker_seek_running:
            marker_status = 'RUNNING'
        elif self.node.last_marker_seek_event not in {'', 'IDLE'}:
            marker_status = f'IDLE (last {self.node.last_marker_seek_event})'
        else:
            marker_status = 'IDLE'
        self.marker_var.set(marker_status)

    def _tick(self):
        if self._closing:
            return
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self._update_status()
        self.root.after(50, self._tick)

    def run(self):
        self.root.focus_force()
        self.root.mainloop()

    def shutdown(self):
        if self._closing:
            return
        self._closing = True

        if rclpy.ok():
            self.node.linear_x = 0.0
            self.node.angular_z = 0.0
            try:
                self.node.publish_cmd_vel()
            except Exception:
                pass

        try:
            self.node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CombinedTeleopNode()
    ui = None
    try:
        ui = CombinedTeleopUI(node)
        ui.run()
    except KeyboardInterrupt:
        pass
    except tk.TclError as exc:
        print(f'Failed to start teleop UI: {exc}')
        print('Ensure DISPLAY is set and a graphical session is available.')
    finally:
        if ui is not None:
            ui.shutdown()
        else:
            node.linear_x = 0.0
            node.angular_z = 0.0
            node.publish_cmd_vel()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
