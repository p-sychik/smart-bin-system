"""Tkinter UI for the Curbie Smart Bin operator workflow."""

from __future__ import annotations

import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import rclpy

from .operator_node import DEFAULT_COLLECTION_MARKER
from .operator_node import DEFAULT_PICKUP_MARKERS
from .operator_node import OperatorNode


BG = '#07110b'
CARD = '#102116'
CARD_ALT = '#152a1b'
ACCENT = '#2ecc71'
ACCENT_DARK = '#1e8747'
TEXT = '#e9fff0'
MUTED = '#94c5a3'
WARN = '#f6c244'


class OperatorUI:
    """Desktop interface for the new Curbie Smart Bin workflow."""

    def __init__(self, node: OperatorNode) -> None:
        self.node = node
        self._closing = False
        self._last_sonar_refresh_s = 0.0
        self._last_marker_detected = False
        self._last_align_phase = 'IDLE'
        self._last_seek_active = False

        self.root = tk.Tk()
        self.root.title('Curbie Smart Bin Operator')
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        width = max(1240, min(1620, screen_w - 40))
        height = max(860, min(980, screen_h - 72))
        self.root.geometry(f'{width}x{height}')
        self.root.minsize(1180, 820)
        self.root.configure(bg=BG, padx=14, pady=14)

        self.path_name_var = tk.StringVar()
        self.path_desc_var = tk.StringVar()
        self.rename_var = tk.StringVar()
        self.motion_var = tk.StringVar()
        self.record_var = tk.StringVar()
        self.replay_var = tk.StringVar()
        self.seek_var = tk.StringVar()
        self.hook_var = tk.StringVar()
        self.offset_var = tk.StringVar()
        self.active_path_var = tk.StringVar()
        self.sonar_summary_var = tk.StringVar()

        self._configure_styles()
        self._build_layout()
        self._refresh_paths()
        self._update_status()

        self.root.bind('<KeyPress>', self._on_keypress)
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)
        self.root.after(60, self._tick)

    def _configure_styles(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        style.configure('.', background=BG, foreground=TEXT, fieldbackground=CARD_ALT)
        style.configure('Title.TLabel', font=('TkDefaultFont', 18, 'bold'), background=BG, foreground=TEXT)
        style.configure('Sub.TLabel', font=('TkDefaultFont', 10), background=BG, foreground=MUTED)
        style.configure('Card.TLabelframe', background=CARD, foreground=TEXT, borderwidth=1)
        style.configure('Card.TLabelframe.Label', background=CARD, foreground=TEXT)
        style.configure('Card.TFrame', background=CARD)
        style.configure('Dark.TFrame', background=BG)
        style.configure('Accent.TButton', background=ACCENT_DARK, foreground=TEXT)
        style.map('Accent.TButton', background=[('active', ACCENT)])
        style.configure('Treeview', background=CARD_ALT, fieldbackground=CARD_ALT, foreground=TEXT)
        style.configure('Treeview.Heading', background=ACCENT_DARK, foreground=TEXT)
        style.map('Treeview', background=[('selected', ACCENT_DARK)])
        style.configure('TNotebook', background=BG, borderwidth=0)
        style.configure('TNotebook.Tab', background=CARD_ALT, foreground=TEXT, padding=(14, 8))
        style.map('TNotebook.Tab', background=[('selected', ACCENT_DARK)])

    def _build_layout(self) -> None:
        title = ttk.Label(self.root, text='Curbie Smart Bin Operator', style='Title.TLabel')
        title.pack(anchor='w')
        subtitle = ttk.Label(
            self.root,
            text=(
                'Drive manually, record reusable paths with embedded seek events, replay any '
                'saved route, and monitor sonar telemetry from one place.'
            ),
            style='Sub.TLabel',
            wraplength=1200,
            justify='left',
        )
        subtitle.pack(anchor='w', pady=(2, 10))

        status = ttk.LabelFrame(self.root, text='Live Status', style='Card.TLabelframe')
        status.pack(fill='x', pady=(0, 10))
        self._build_status(status)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True)

        self.drive_tab = ttk.Frame(notebook, style='Dark.TFrame', padding=10)
        self.paths_tab = ttk.Frame(notebook, style='Dark.TFrame', padding=10)
        self.seek_tab = ttk.Frame(notebook, style='Dark.TFrame', padding=10)

        notebook.add(self.drive_tab, text='Drive')
        notebook.add(self.paths_tab, text='Paths')
        notebook.add(self.seek_tab, text='Seek + Sonar')

        self._build_drive_tab()
        self._build_paths_tab()
        self._build_seek_tab()

    def _build_status(self, parent: ttk.LabelFrame) -> None:
        for column in range(4):
            parent.columnconfigure(column, weight=1)
        rows = [
            ('Motion', self.motion_var),
            ('Recording', self.record_var),
            ('Replay', self.replay_var),
            ('Seek', self.seek_var),
            ('Hook', self.hook_var),
            ('Collection Offset', self.offset_var),
            ('Active Path', self.active_path_var),
            ('Sonar (monitor only)', self.sonar_summary_var),
        ]
        for index, (label, variable) in enumerate(rows):
            row = index // 2
            column = (index % 2) * 2
            ttk.Label(parent, text=f'{label}:').grid(row=row, column=column, sticky='w', padx=10, pady=4)
            ttk.Label(parent, textvariable=variable).grid(row=row, column=column + 1, sticky='w', padx=(0, 12), pady=4)

    def _build_drive_tab(self) -> None:
        self.drive_tab.columnconfigure(0, weight=3)
        self.drive_tab.columnconfigure(1, weight=2)
        self.drive_tab.rowconfigure(0, weight=1)

        controls = ttk.LabelFrame(self.drive_tab, text='Drive + Hooks', style='Card.TLabelframe')
        controls.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)

        log_frame = ttk.LabelFrame(self.drive_tab, text='Activity Log', style='Card.TLabelframe')
        log_frame.grid(row=0, column=1, sticky='nsew')
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self._button(controls, 'Forward (W)', lambda: self._dispatch_key('w')).grid(row=0, column=0, sticky='ew', padx=6, pady=6)
        self._button(controls, 'Backward (S)', lambda: self._dispatch_key('s')).grid(row=0, column=1, sticky='ew', padx=6, pady=6)
        self._button(controls, 'Turn Left (A)', lambda: self._dispatch_key('a')).grid(row=1, column=0, sticky='ew', padx=6, pady=6)
        self._button(controls, 'Turn Right (D)', lambda: self._dispatch_key('d')).grid(row=1, column=1, sticky='ew', padx=6, pady=6)
        self._button(controls, 'Stop / Cancel (X)', lambda: self._dispatch_key('x')).grid(row=2, column=0, columnspan=2, sticky='ew', padx=6, pady=6)
        self._button(controls, 'Engage Hook (V)', lambda: self._dispatch_key('v')).grid(row=3, column=0, sticky='ew', padx=6, pady=6)
        self._button(controls, 'Release Hook (C)', lambda: self._dispatch_key('c')).grid(row=3, column=1, sticky='ew', padx=6, pady=6)

        shortcuts = tk.Label(
            controls,
            text='Keyboard: W/A/S/D move, X stop + cancel, V engage, C release, 1 bin 10, 2 bin 11, G dropoff 40, Space replay selected path.',
            bg=CARD,
            fg=MUTED,
            justify='left',
            wraplength=500,
        )
        shortcuts.grid(row=4, column=0, columnspan=2, sticky='w', padx=10, pady=(8, 10))

        self.log_text = tk.Text(
            log_frame,
            bg=CARD_ALT,
            fg=TEXT,
            insertbackground=TEXT,
            borderwidth=0,
            highlightthickness=0,
            wrap='word',
        )
        self.log_text.grid(row=0, column=0, sticky='nsew', padx=(8, 0), pady=8)
        log_scroll = ttk.Scrollbar(log_frame, orient='vertical', command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky='ns', padx=(0, 8), pady=8)
        self.log_text.configure(yscrollcommand=log_scroll.set, state='disabled')

    def _build_paths_tab(self) -> None:
        self.paths_tab.columnconfigure(0, weight=1)
        self.paths_tab.columnconfigure(1, weight=1)
        self.paths_tab.rowconfigure(1, weight=1)

        record_card = ttk.LabelFrame(self.paths_tab, text='Record New Path', style='Card.TLabelframe')
        record_card.grid(row=0, column=0, sticky='ew', padx=(0, 8), pady=(0, 8))
        record_card.columnconfigure(1, weight=1)

        ttk.Label(record_card, text='Path Name').grid(row=0, column=0, sticky='w', padx=8, pady=6)
        ttk.Entry(record_card, textvariable=self.path_name_var).grid(row=0, column=1, sticky='ew', padx=8, pady=6)
        ttk.Label(record_card, text='Description').grid(row=1, column=0, sticky='w', padx=8, pady=6)
        ttk.Entry(record_card, textvariable=self.path_desc_var).grid(row=1, column=1, sticky='ew', padx=8, pady=6)

        row = ttk.Frame(record_card, style='Card.TFrame')
        row.grid(row=2, column=0, columnspan=2, sticky='ew', padx=8, pady=(0, 8))
        row.columnconfigure(0, weight=1)
        row.columnconfigure(1, weight=1)
        self._button(row, 'Start Recording', self._start_recording).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        self._button(row, 'Stop Recording', self._stop_recording).grid(row=0, column=1, sticky='ew', padx=(4, 0))

        tips = ttk.LabelFrame(self.paths_tab, text='Record Event Buttons', style='Card.TLabelframe')
        tips.grid(row=0, column=1, sticky='ew', pady=(0, 8))
        tips.columnconfigure(0, weight=1)
        tips.columnconfigure(1, weight=1)
        self._button(tips, 'Record + Run Pickup 10', lambda: self._start_pickup(10)).grid(row=0, column=0, sticky='ew', padx=8, pady=8)
        self._button(tips, 'Record + Run Pickup 11', lambda: self._start_pickup(11)).grid(row=0, column=1, sticky='ew', padx=8, pady=8)
        self._button(tips, 'Record + Run Dropoff 40', self._start_dropoff).grid(row=1, column=0, columnspan=2, sticky='ew', padx=8, pady=(0, 8))

        library = ttk.LabelFrame(self.paths_tab, text='Saved Paths', style='Card.TLabelframe')
        library.grid(row=1, column=0, columnspan=2, sticky='nsew')
        library.columnconfigure(0, weight=1)
        library.rowconfigure(0, weight=1)

        columns = ('name', 'steps', 'points', 'updated')
        self.path_tree = ttk.Treeview(library, columns=columns, show='headings', height=14)
        self.path_tree.grid(row=0, column=0, sticky='nsew', padx=(8, 0), pady=8)
        for col, title, width in (
            ('name', 'Path', 260),
            ('steps', 'Steps', 80),
            ('points', 'Points', 80),
            ('updated', 'Updated', 160),
        ):
            self.path_tree.heading(col, text=title)
            self.path_tree.column(col, width=width, stretch=(col == 'name'))
        self.path_tree.bind('<<TreeviewSelect>>', self._on_path_selected)
        self.path_tree.bind('<Double-Button-1>', lambda _event: self._replay_selected_path())

        tree_scroll = ttk.Scrollbar(library, orient='vertical', command=self.path_tree.yview)
        tree_scroll.grid(row=0, column=1, sticky='ns', padx=(0, 8), pady=8)
        self.path_tree.configure(yscrollcommand=tree_scroll.set)

        actions = ttk.Frame(library, style='Card.TFrame')
        actions.grid(row=1, column=0, columnspan=2, sticky='ew', padx=8, pady=(0, 8))
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        actions.columnconfigure(2, weight=1)
        actions.columnconfigure(3, weight=1)
        ttk.Entry(actions, textvariable=self.rename_var).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        self._button(actions, 'Rename', self._rename_selected_path).grid(row=0, column=1, sticky='ew', padx=4)
        self._button(actions, 'Replay Selected', self._replay_selected_path).grid(row=0, column=2, sticky='ew', padx=4)
        self._button(actions, 'Delete', self._delete_selected_path).grid(row=0, column=3, sticky='ew', padx=(4, 0))

        footer = ttk.Frame(library, style='Card.TFrame')
        footer.grid(row=2, column=0, columnspan=2, sticky='ew', padx=8, pady=(0, 8))
        footer.columnconfigure(0, weight=1)
        footer.columnconfigure(1, weight=1)
        self._button(footer, 'Refresh Paths', self._refresh_paths).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        self._button(footer, 'Cancel Replay / Seek', lambda: self._dispatch_key('x')).grid(row=0, column=1, sticky='ew', padx=(4, 0))

    def _build_seek_tab(self) -> None:
        self.seek_tab.columnconfigure(0, weight=1)
        self.seek_tab.columnconfigure(1, weight=1)

        seek_card = ttk.LabelFrame(self.seek_tab, text='Marker Seek Actions', style='Card.TLabelframe')
        seek_card.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        seek_card.columnconfigure(0, weight=1)
        seek_card.columnconfigure(1, weight=1)

        for index, marker_id in enumerate(DEFAULT_PICKUP_MARKERS):
            self._button(
                seek_card,
                f'Pickup Bin Marker {marker_id}',
                lambda marker_id=marker_id: self._start_pickup(marker_id),
            ).grid(row=index, column=0, columnspan=2, sticky='ew', padx=8, pady=8)

        self._button(
            seek_card,
            f'Dropoff To Marker {DEFAULT_COLLECTION_MARKER}',
            self._start_dropoff,
        ).grid(row=2, column=0, columnspan=2, sticky='ew', padx=8, pady=8)
        self._button(seek_card, 'Reset Collection Offset', self._reset_offset).grid(row=3, column=0, sticky='ew', padx=8, pady=8)
        self._button(seek_card, 'Cancel Active Seek / Replay', lambda: self._dispatch_key('x')).grid(row=3, column=1, sticky='ew', padx=8, pady=8)

        sonar_card = ttk.LabelFrame(self.seek_tab, text='Five-Sonar Readout', style='Card.TLabelframe')
        sonar_card.grid(row=0, column=1, sticky='nsew')
        sonar_card.columnconfigure(1, weight=1)

        self.sonar_labels = {}
        rows = [
            ('Sonar 1', 'Right / East'),
            ('Sonar 2', 'Front-Left / North-West'),
            ('Sonar 3', 'Left / West'),
            ('Sonar 4', 'Front-Right / North-East'),
            ('Sonar 5', 'Front / North'),
        ]
        for row_index, (name, desc) in enumerate(rows):
            ttk.Label(sonar_card, text=f'{name}:').grid(row=row_index, column=0, sticky='w', padx=8, pady=6)
            label = ttk.Label(sonar_card, text='--')
            label.grid(row=row_index, column=1, sticky='w', padx=(0, 8), pady=6)
            ttk.Label(sonar_card, text=desc, style='Sub.TLabel').grid(row=row_index, column=2, sticky='w', padx=(0, 8), pady=6)
            self.sonar_labels[row_index] = label

        note = tk.Label(
            sonar_card,
            text=(
                'The UI refreshes the displayed distances once per second. '
                'Sonar is telemetry-only in this build and is not used for obstacle avoidance.'
            ),
            bg=CARD,
            fg=MUTED,
            justify='left',
            wraplength=480,
        )
        note.grid(row=5, column=0, columnspan=3, sticky='w', padx=8, pady=(6, 8))

    def _button(self, parent, text, command):
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg=ACCENT_DARK,
            fg=TEXT,
            activebackground=ACCENT,
            activeforeground=TEXT,
            relief='flat',
            padx=10,
            pady=8,
            highlightthickness=0,
        )

    def _append_log(self, message: str) -> None:
        if not message:
            return
        stamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.configure(state='normal')
        self.log_text.insert('end', f'[{stamp}] {message}\n')
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    def _start_recording(self) -> None:
        name = self.path_name_var.get().strip()
        desc = self.path_desc_var.get().strip()
        ok, message = self.node.start_recording(name, desc)
        self._append_log(message)
        self._update_status()
        if ok:
            self.rename_var.set('')

    def _stop_recording(self) -> None:
        ok, message = self.node.stop_recording()
        self._append_log(message)
        self._update_status()
        if ok:
            self._refresh_paths(select_id=self.node.last_completed_path_id)

    def _refresh_paths(self, select_id: str | None = None) -> None:
        current = select_id or self.node.selected_path_id
        self.path_tree.delete(*self.path_tree.get_children())
        for summary in self.node.list_saved_paths():
            updated = datetime.fromtimestamp(summary.updated_at_s).strftime('%Y-%m-%d %H:%M')
            self.path_tree.insert(
                '',
                'end',
                iid=summary.path_id,
                values=(
                    summary.display_name,
                    f'{summary.step_count} ({summary.event_step_count} events)',
                    str(summary.motion_point_count),
                    updated,
                ),
            )
        if current and self.path_tree.exists(current):
            self.path_tree.selection_set(current)
            self.path_tree.focus(current)
            self.node.set_selected_path(current)

    def _on_path_selected(self, _event=None) -> None:
        selection = self.path_tree.selection()
        if not selection:
            return
        path_id = selection[0]
        self.node.set_selected_path(path_id)
        self.rename_var.set(path_id)

    def _rename_selected_path(self) -> None:
        selection = self.path_tree.selection()
        if not selection:
            self._append_log('Select a path first.')
            return
        ok, message = self.node.rename_saved_path(selection[0], self.rename_var.get().strip())
        self._append_log(message)
        if ok:
            self._refresh_paths(select_id=self.node.selected_path_id)

    def _delete_selected_path(self) -> None:
        selection = self.path_tree.selection()
        if not selection:
            self._append_log('Select a path first.')
            return
        ok, message = self.node.delete_saved_path(selection[0])
        self._append_log(message)
        if ok:
            self._refresh_paths()

    def _replay_selected_path(self) -> None:
        selection = self.path_tree.selection()
        if not selection:
            self._append_log('Select a path first.')
            return
        ok, message = self.node.replay_path(selection[0])
        self._append_log(message)
        self._update_status()

    def _start_pickup(self, marker_id: int) -> None:
        ok, message = self.node.start_pickup_seek(marker_id)
        self._append_log(message)
        self._update_status()

    def _start_dropoff(self) -> None:
        ok, message = self.node.start_dropoff_seek()
        self._append_log(message)
        self._update_status()

    def _reset_offset(self) -> None:
        ok, message = self.node.reset_collection_offset()
        self._append_log(message)
        self._update_status()

    def _dispatch_key(self, key: str) -> None:
        keep_running, message = self.node.handle_key(key)
        if message:
            self._append_log(message)
        self._update_status()
        if keep_running is False:
            self.shutdown()

    def _is_typing_widget_focused(self) -> bool:
        widget = self.root.focus_get()
        if widget is None:
            return False
        return widget.winfo_class() in {
            'Entry',
            'TEntry',
            'Text',
            'Spinbox',
            'TCombobox',
        }

    def _on_keypress(self, event) -> None:
        if self._is_typing_widget_focused():
            return

        if event.keysym == 'space':
            key = ' '
        elif event.char:
            key = event.char
        else:
            return
        allowed = {'w', 'a', 's', 'd', 'x', 'c', 'v', '1', '2', 'g', 'o', 'p', 'q'}
        if key.lower() in allowed or key == ' ':
            self._dispatch_key(key)

    def _update_status(self) -> None:
        self.motion_var.set(
            f'target linear={self.node.linear_x:.2f} m/s, angular={self.node.angular_z:.2f} rad/s '
            f'| applied linear={self.node.applied_linear_x:.2f} m/s, angular={self.node.applied_angular_z:.2f} rad/s'
        )
        self.record_var.set(self.node.recording_status)
        self.replay_var.set(f'{self.node.replay_status} | sonar: {self.node.last_avoidance_note}')
        marker_text = (
            str(self.node.align_target_marker_id)
            if self.node.align_target_marker_id is not None
            else '-'
        )
        visibility = 'VISIBLE' if self.node.align_surface_detected else 'NOT VISIBLE'
        distance_text = (
            f'{self.node.align_distance_to_surface_m:.2f} m'
            if self.node.align_surface_detected
            else '--'
        )
        self.seek_var.set(
            f'{self.node.seek_status} | phase: {self.node.align_feedback_phase} | '
            f'marker: {marker_text} ({visibility}) | distance: {distance_text}'
        )
        self.hook_var.set(f'{self.node.hook_state} | attached={self.node.hook_attached}')
        self.offset_var.set(f'{self.node.collection_drop_count * self.node.dropoff_lateral_spacing_m:.2f} m left')
        self.active_path_var.set(self.node.active_path_id or '-')
        self.sonar_summary_var.set(self.node.sonar_status_text)
        self._maybe_log_marker_detection()
        self._maybe_log_align_phase_transition()
        self._maybe_log_seek_completion()

        now_s = time.monotonic()
        if now_s - self._last_sonar_refresh_s >= 1.0:
            for index, value in enumerate(self.node.sonar_distances_m):
                if index in self.sonar_labels:
                    self.sonar_labels[index].configure(text=f'{value:.2f} m')
            self._last_sonar_refresh_s = now_s

    def _maybe_log_marker_detection(self) -> None:
        if not self.node.seek_active and not self.node.replay_active:
            self._last_marker_detected = False
            return

        detected = bool(self.node.align_surface_detected)
        marker_id = self.node.align_target_marker_id
        marker_text = str(marker_id) if marker_id is not None else '?'
        if detected and not self._last_marker_detected:
            self._append_log(
                'Marker '
                f'{marker_text} detected '
                f'(phase={self.node.align_feedback_phase}, '
                f'z={self.node.align_distance_to_surface_m:.2f} m, '
                f'x_err={self.node.align_x_error_m:+.2f} m, '
                f'yaw_err={self.node.align_yaw_error_rad:+.2f} rad).'
            )
        elif (not detected) and self._last_marker_detected:
            self._append_log(
                f'Marker {marker_text} moved out of frame '
                f'(phase={self.node.align_feedback_phase}); continuing the current '
                '720-degree search sweep.'
            )
        self._last_marker_detected = detected

    def _maybe_log_align_phase_transition(self) -> None:
        active = self.node.seek_active or self.node.replay_active
        if not active:
            self._last_align_phase = 'IDLE'
            return

        phase = str(self.node.align_feedback_phase or 'UNKNOWN')
        if phase == self._last_align_phase:
            return

        marker_text = (
            str(self.node.align_target_marker_id)
            if self.node.align_target_marker_id is not None
            else '?'
        )
        if phase in {'SEARCHING', 'LOST_SCAN'}:
            if phase == 'LOST_SCAN' and self.node.align_mode == 'pickup':
                self._append_log(
                    f'Seek phase LOST_SCAN: rear camera is continuing its 720-degree '
                    f'sweep for marker {marker_text}; the next full sweep will reverse direction.'
                )
            elif phase == 'LOST_SCAN' and self.node.align_mode == 'dropoff':
                self._append_log(
                    'Seek phase LOST_SCAN: front camera is continuing its 720-degree '
                    f'sweep for collection marker {marker_text}; the next full sweep will reverse direction.'
                )
            elif self.node.align_mode == 'pickup':
                self._append_log(
                    f'Seek phase {phase}: rear camera performing a 720-degree sweep '
                    f'to find marker {marker_text}.'
                )
            elif self.node.align_mode == 'dropoff':
                self._append_log(
                    f'Seek phase {phase}: front camera performing a 720-degree sweep '
                    f'to find collection marker {marker_text}.'
                )
            else:
                self._append_log(
                    f'Seek phase {phase}: performing a 720-degree sweep to find marker {marker_text}.'
                )
        elif phase in {'COARSE_ALIGN', 'FINE_ALIGN'}:
            self._append_log(
                f'Seek phase {phase}: marker {marker_text} visible, navigating '
                f'(z={self.node.align_distance_to_surface_m:.2f} m).'
            )
        elif phase == 'HOLDING':
            self._append_log(
                f'Seek phase HOLDING: close to marker {marker_text}, final stabilization.'
            )
        elif phase == 'HOOK_ENGAGING':
            self._append_log(f'Pickup marker {marker_text} aligned; engaging hook.')
        elif phase == 'PICKUP_COMPLETE':
            self._append_log(
                f'Pickup complete for marker {marker_text}: hook engaged and bin secured.'
            )
        elif phase == 'HOOK_RELEASING':
            self._append_log('Collection marker aligned; releasing hook.')
        elif phase == 'DROPOFF_COMPLETE':
            self._append_log('Dropoff complete: hook released.')
        elif phase == 'SUCCEEDED':
            if self.node.last_seek_mode == 'pickup' or self.node.current_seek_mode == 'pickup':
                self._append_log(
                    f'Pickup marker {marker_text} aligned; hook engagement is next.'
                )
            elif self.node.last_seek_mode == 'dropoff' or self.node.current_seek_mode == 'dropoff':
                self._append_log(
                    f'Collection marker {marker_text} aligned; hook release is next.'
                )
            else:
                self._append_log(f'Seek phase SUCCEEDED for marker {marker_text}.')
        elif phase == 'FAILED':
            if self.node.last_seek_mode == 'pickup':
                self._append_log(
                    f'Seek phase FAILED reached internally for marker {marker_text}; '
                    'pickup search is continuing.'
                )
            else:
                self._append_log(f'Seek phase FAILED for marker {marker_text}.')
        elif phase == 'OBSTACLE_WAIT':
            self._append_log('Seek paused: obstacle wait.')
        elif phase == 'HOOK_ENGAGE_FAILED':
            self._append_log(
                f'Pickup alignment reached marker {marker_text}, but hook engagement failed.'
            )
        elif phase == 'HOOK_RELEASE_FAILED':
            self._append_log('Dropoff alignment succeeded, but hook release failed.')

        self._last_align_phase = phase

    def _maybe_log_seek_completion(self) -> None:
        active = bool(self.node.seek_active)
        if self._last_seek_active and not active:
            if self.node.last_seek_mode == 'pickup':
                if self.node.last_seek_success and self.node.hook_attached:
                    self._append_log(
                        'Pickup seek complete: close to marker and hook engaged.'
                    )
                elif not self.node.last_seek_success:
                    self._append_log(
                        'Pickup seek stopped before hook engagement was completed.'
                    )
            elif self.node.last_seek_mode == 'dropoff':
                if self.node.last_seek_success:
                    self._append_log('Dropoff seek complete.')
                else:
                    self._append_log('Dropoff seek stopped before completion.')
        self._last_seek_active = active

    def _tick(self) -> None:
        if self._closing:
            return
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self._update_status()
        self.root.after(60, self._tick)

    def run(self) -> None:
        self.root.focus_force()
        self.root.mainloop()

    def shutdown(self) -> None:
        if self._closing:
            return
        self._closing = True
        try:
            self.node.shutdown()
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OperatorNode()
    ui = None
    try:
        ui = OperatorUI(node)
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        if ui is not None:
            ui.shutdown()
        else:
            node.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
