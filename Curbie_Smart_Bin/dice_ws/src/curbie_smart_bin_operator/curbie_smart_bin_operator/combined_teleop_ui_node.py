import json
from datetime import datetime
import tkinter as tk
from tkinter import ttk
import urllib.error
import urllib.request

import rclpy

from .combined_teleop_node import CombinedTeleopNode


class CombinedTeleopUI:
    def __init__(self, node):
        self.node = node
        self._closing = False
        self._bins = []
        self._last_saved_path_id = ''

        self.root = tk.Tk()
        self.root.title('TurtleBot3 Teleop + Mission Setup')
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        window_width = max(1100, min(1520, screen_width - 32))
        window_height = max(780, min(960, screen_height - 72))
        self.root.geometry(f'{window_width}x{window_height}')
        self.root.minsize(1040, 760)
        self.root.configure(padx=14, pady=14)

        self.motion_var = tk.StringVar()
        self.servo_var = tk.StringVar()
        self.hook_var = tk.StringVar()
        self.recording_var = tk.StringVar()
        self.marker_var = tk.StringVar()
        self.record_count_var = tk.StringVar()
        self.path_status_var = tk.StringVar(value='Idle')
        self.last_saved_path_var = tk.StringVar(value='No saved path yet.')
        self.bin_status_var = tk.StringVar(value='No bins loaded yet.')
        self.mission_request_var = tk.StringVar(value='No mission queued yet.')
        self.active_mission_id_var = tk.StringVar(value='-')
        self.active_mission_bin_var = tk.StringVar(value='-')
        self.mission_state_var = tk.StringVar(value='IDLE')
        self.mission_action_var = tk.StringVar(value='-')
        self.mission_progress_var = tk.StringVar(value='0.0%')
        self.mission_error_var = tk.StringVar(value='None')
        self.front_camera_status_var = tk.StringVar(
            value='Waiting for /curbie/front_camera/image_raw'
        )
        self.rear_camera_status_var = tk.StringVar(
            value='Waiting for /curbie/rear_camera/image_raw'
        )

        self.backend_url_var = tk.StringVar(value='http://127.0.0.1:8000')
        self.path_id_var = tk.StringVar()
        self.path_description_var = tk.StringVar()
        self.rfid_tag_var = tk.StringVar()
        self.bin_type_var = tk.StringVar(value='waste')
        self.location_name_var = tk.StringVar()
        self.path_to_bin_var = tk.StringVar()
        self.path_to_collection_var = tk.StringVar()
        self.path_to_home_var = tk.StringVar()
        self.pickup_marker_var = tk.StringVar()
        self.selected_bin_var = tk.StringVar()
        self._front_camera_photo = None
        self._rear_camera_photo = None
        self._front_camera_seq = -1
        self._rear_camera_seq = -1

        self._build_layout()
        self._update_status()

        self.root.bind('<KeyPress>', self._on_keypress)
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)
        self.root.after(50, self._tick)
        self.root.after(250, self._initial_refresh)

    def _build_layout(self):
        title = ttk.Label(
            self.root,
            text='Combined TurtleBot3 Teleop',
            font=('TkDefaultFont', 16, 'bold'),
        )
        title.pack(anchor='w')

        subtitle = ttk.Label(
            self.root,
            text=(
                'Keyboard shortcuts work from any tab. Use the pages below to drive, '
                'record paths, register bins, inspect cameras, and queue missions.'
            ),
            wraplength=1100,
        )
        subtitle.pack(anchor='w', pady=(0, 8))

        content = ttk.Frame(self.root)
        content.pack(fill='both', expand=True)
        content.columnconfigure(0, weight=1)
        content.rowconfigure(1, weight=1)

        status = ttk.LabelFrame(content, text='Live Status')
        status.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        self._build_status_panel(status)

        notebook = ttk.Notebook(content)
        notebook.grid(row=1, column=0, sticky='nsew')

        drive_tab = ttk.Frame(notebook, padding=10)
        setup_tab = ttk.Frame(notebook, padding=10)
        queue_tab = ttk.Frame(notebook, padding=10)
        camera_tab = ttk.Frame(notebook, padding=10)

        notebook.add(drive_tab, text='Drive + Log')
        notebook.add(setup_tab, text='Bin Setup')
        notebook.add(queue_tab, text='Mission Queue')
        notebook.add(camera_tab, text='Cameras')

        self._build_drive_tab(drive_tab)
        self._build_setup_tab(setup_tab)
        self._build_queue_tab(queue_tab)
        self._build_camera_tab(camera_tab)

    def _build_drive_tab(self, parent):
        parent.columnconfigure(0, weight=3)
        parent.columnconfigure(1, weight=2)
        parent.rowconfigure(0, weight=1)

        controls = ttk.LabelFrame(parent, text='Drive + Hook Controls')
        controls.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)

        activity = ttk.LabelFrame(parent, text='Activity Log')
        activity.grid(row=0, column=1, sticky='nsew')
        activity.columnconfigure(0, weight=1)
        activity.rowconfigure(0, weight=1)

        tip = ttk.Label(
            parent,
            text=(
                'Tip: the keyboard shortcuts still work while you are on the Setup, '
                'Mission Queue, or Cameras tabs.'
            ),
            wraplength=1000,
        )
        tip.grid(row=1, column=0, columnspan=2, sticky='w', pady=(8, 0))

        self._build_controls_panel(controls)
        self._build_activity_panel(activity)

    def _build_setup_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)

        scroll_frame = self._create_scrollable_frame(parent)
        scroll_frame.columnconfigure(0, weight=1)

        intro = ttk.Label(
            scroll_frame,
            text=(
                'Record a path, apply it to the relevant field, then create the bin '
                'entry in the backend. This page scrolls if the form is taller than '
                'the window.'
            ),
            wraplength=900,
            justify='left',
        )
        intro.grid(row=0, column=0, sticky='w', padx=4, pady=(0, 8))

        setup_panel = ttk.Frame(scroll_frame)
        setup_panel.grid(row=1, column=0, sticky='ew')
        setup_panel.columnconfigure(0, weight=1)
        self._build_setup_panel(setup_panel)

    def _build_queue_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        self._build_mission_panel(parent)

    def _build_camera_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        intro = ttk.Label(
            parent,
            text='Live front and rear camera feeds from the robot.',
        )
        intro.grid(row=0, column=0, sticky='w', pady=(0, 8))

        cameras = ttk.LabelFrame(parent, text='Camera Feeds')
        cameras.grid(row=1, column=0, sticky='nsew')
        cameras.columnconfigure(0, weight=1)
        cameras.columnconfigure(1, weight=1)
        self._build_camera_panel(cameras)

    def _create_scrollable_frame(self, parent):
        container = ttk.Frame(parent)
        container.grid(row=0, column=0, sticky='nsew')
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)

        canvas = tk.Canvas(
            container,
            borderwidth=0,
            highlightthickness=0,
        )
        canvas.grid(row=0, column=0, sticky='nsew')

        scrollbar = ttk.Scrollbar(
            container,
            orient='vertical',
            command=canvas.yview,
        )
        scrollbar.grid(row=0, column=1, sticky='ns')
        canvas.configure(yscrollcommand=scrollbar.set)

        inner = ttk.Frame(canvas)
        window_id = canvas.create_window((0, 0), window=inner, anchor='nw')

        def _sync_scroll_region(_event=None):
            canvas.configure(scrollregion=canvas.bbox('all'))

        def _sync_inner_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        def _on_mousewheel(event):
            if event.delta:
                step = -1 if event.delta > 0 else 1
            elif getattr(event, 'num', None) == 4:
                step = -1
            else:
                step = 1
            canvas.yview_scroll(step, 'units')

        def _bind_mousewheel(_event=None):
            canvas.bind_all('<MouseWheel>', _on_mousewheel)
            canvas.bind_all('<Button-4>', _on_mousewheel)
            canvas.bind_all('<Button-5>', _on_mousewheel)

        def _unbind_mousewheel(_event=None):
            canvas.unbind_all('<MouseWheel>')
            canvas.unbind_all('<Button-4>')
            canvas.unbind_all('<Button-5>')

        inner.bind('<Configure>', _sync_scroll_region)
        canvas.bind('<Configure>', _sync_inner_width)
        canvas.bind('<Enter>', _bind_mousewheel)
        canvas.bind('<Leave>', _unbind_mousewheel)
        return inner

    def _build_controls_panel(self, parent):
        self._add_section_label(parent, 'Movement', row=0)
        self._add_button(parent, 'Forward (W)', 'w', row=1, col=0)
        self._add_button(parent, 'Backward (S)', 's', row=1, col=1)
        self._add_button(parent, 'Turn Left (A)', 'a', row=2, col=0)
        self._add_button(parent, 'Turn Right (D)', 'd', row=2, col=1)
        self._add_button(parent, 'Stop + Cancel Seek (X)', 'x',
                         row=3, col=0, colspan=2)

        self._add_section_label(parent, 'Servos', row=4)
        self._add_button(parent, 'Unhook (C)', 'c', row=5, col=0)
        self._add_button(parent, 'Hook (V)', 'v', row=5, col=1)

        self._add_section_label(parent, 'Recording', row=6)
        self._add_button(parent, 'Start Recording (O)', 'o', row=7, col=0)
        self._add_button(parent, 'Stop Recording (P)', 'p', row=7, col=1)
        self._add_button(parent, 'Replay (Space)', ' ', row=8, col=0)
        self._add_button(parent, 'Run Test Sequence (T)', 't', row=8, col=1)

        self._add_section_label(parent, 'Marker Seek', row=9)
        self._add_button(parent, 'Start Seek (F)', 'f', row=10, col=0)
        self._add_button(parent, 'Quit (Q)', 'q', row=10, col=1)

    def _build_status_panel(self, parent):
        parent.columnconfigure(1, weight=1)
        parent.columnconfigure(3, weight=1)
        status_rows = [
            ('Motion:', self.motion_var),
            ('Servos:', self.servo_var),
            ('Hook:', self.hook_var),
            ('Recording:', self.recording_var),
            ('Recorded Actions:', self.record_count_var),
            ('Marker Seek:', self.marker_var),
            ('Path Recorder:', self.path_status_var),
            ('Last Saved Path:', self.last_saved_path_var),
            ('Bin Setup:', self.bin_status_var),
            ('Mission Queue:', self.mission_request_var),
        ]
        for row, (label, variable) in enumerate(status_rows):
            grid_column = 0 if row < 5 else 2
            grid_row = row if row < 5 else row - 5
            ttk.Label(parent, text=label).grid(
                row=grid_row,
                column=grid_column,
                sticky='w',
                padx=8,
                pady=3,
            )
            ttk.Label(parent, textvariable=variable).grid(
                row=grid_row,
                column=grid_column + 1,
                sticky='w',
                padx=(0, 8),
                pady=3,
            )

    def _build_setup_panel(self, parent):
        parent.columnconfigure(0, weight=1)
        recorder = ttk.LabelFrame(parent, text='Path Recorder')
        recorder.grid(row=0, column=0, sticky='nsew', padx=8, pady=(8, 6))
        recorder.columnconfigure(1, weight=1)

        ttk.Label(recorder, text='Path ID').grid(
            row=0, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(recorder, textvariable=self.path_id_var).grid(
            row=0, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(recorder, text='Description').grid(
            row=1, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(recorder, textvariable=self.path_description_var).grid(
            row=1, column=1, sticky='ew', padx=8, pady=4)

        button_row = ttk.Frame(recorder)
        button_row.grid(row=2, column=0, columnspan=2, sticky='ew', padx=8, pady=4)
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        ttk.Button(
            button_row,
            text='Start Path Recording',
            command=self._start_path_recording,
        ).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(
            button_row,
            text='Stop Path Recording',
            command=self._stop_path_recording,
        ).grid(row=0, column=1, sticky='ew', padx=(4, 0))

        fill_row = ttk.Frame(recorder)
        fill_row.grid(row=3, column=0, columnspan=2, sticky='ew', padx=8, pady=(0, 6))
        fill_row.columnconfigure(0, weight=1)
        fill_row.columnconfigure(1, weight=1)
        fill_row.columnconfigure(2, weight=1)
        ttk.Button(
            fill_row,
            text='Use Last Path for Path To Bin',
            command=lambda: self._apply_last_path(self.path_to_bin_var, 'path_to_bin'),
        ).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(
            fill_row,
            text='Use Last Path for Path To Collection',
            command=lambda: self._apply_last_path(
                self.path_to_collection_var,
                'path_to_collection',
            ),
        ).grid(row=0, column=1, sticky='ew', padx=4)
        ttk.Button(
            fill_row,
            text='Use Last Path for Path To Home',
            command=lambda: self._apply_last_path(self.path_to_home_var, 'path_to_home'),
        ).grid(row=0, column=2, sticky='ew', padx=(4, 0))

        registration = ttk.LabelFrame(parent, text='Bin Registration')
        registration.grid(row=1, column=0, sticky='nsew', padx=8, pady=(6, 8))
        registration.columnconfigure(1, weight=1)

        ttk.Label(registration, text='Backend URL').grid(
            row=0, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(registration, textvariable=self.backend_url_var).grid(
            row=0, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(registration, text='RFID Tag ID').grid(
            row=1, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(registration, textvariable=self.rfid_tag_var).grid(
            row=1, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(registration, text='Bin Type').grid(
            row=2, column=0, sticky='w', padx=8, pady=4)
        ttk.Combobox(
            registration,
            textvariable=self.bin_type_var,
            values=('waste', 'recycling', 'compost'),
            state='readonly',
        ).grid(row=2, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(registration, text='Location Name').grid(
            row=3, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(registration, textvariable=self.location_name_var).grid(
            row=3, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(registration, text='Path To Bin ID').grid(
            row=4, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(registration, textvariable=self.path_to_bin_var).grid(
            row=4, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(registration, text='Path To Collection ID').grid(
            row=5, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(registration, textvariable=self.path_to_collection_var).grid(
            row=5, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(registration, text='Path To Home ID').grid(
            row=6, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(registration, textvariable=self.path_to_home_var).grid(
            row=6, column=1, sticky='ew', padx=8, pady=4)

        ttk.Label(registration, text='Pickup Marker ID').grid(
            row=7, column=0, sticky='w', padx=8, pady=4)
        ttk.Entry(registration, textvariable=self.pickup_marker_var).grid(
            row=7, column=1, sticky='ew', padx=8, pady=4)

        create_row = ttk.Frame(registration)
        create_row.grid(row=8, column=0, columnspan=2, sticky='ew', padx=8, pady=(4, 8))
        create_row.columnconfigure(0, weight=1)
        create_row.columnconfigure(1, weight=1)
        ttk.Button(
            create_row,
            text='Create Bin in Backend',
            command=self._create_bin,
        ).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(
            create_row,
            text='Refresh Bin List',
            command=self._refresh_bins,
        ).grid(row=0, column=1, sticky='ew', padx=(4, 0))

    def _build_mission_panel(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        header = ttk.Frame(parent)
        header.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        header.columnconfigure(0, weight=1)
        ttk.Label(
            header,
            text='Select a registered bin, refresh if needed, then queue it into the multi-bin mission controller.',
            wraplength=900,
            justify='left',
        ).grid(row=0, column=0, sticky='w')
        ttk.Button(
            header,
            text='Refresh Bin List',
            command=self._refresh_bins,
        ).grid(row=0, column=1, sticky='e', padx=(8, 0))

        list_frame = ttk.LabelFrame(parent, text='Registered Bins')
        list_frame.grid(row=1, column=0, sticky='nsew')
        list_frame.columnconfigure(0, weight=1)
        list_frame.rowconfigure(0, weight=1)

        self.bin_listbox = tk.Listbox(
            list_frame,
            exportselection=False,
            height=14,
        )
        self.bin_listbox.grid(row=0, column=0, sticky='nsew', padx=(8, 0), pady=8)
        self.bin_listbox.bind('<<ListboxSelect>>', self._on_bin_selected)
        self.bin_listbox.bind('<Double-Button-1>', lambda _event: self._queue_mission())

        scrollbar = ttk.Scrollbar(
            list_frame,
            orient='vertical',
            command=self.bin_listbox.yview,
        )
        scrollbar.grid(row=0, column=1, sticky='ns', padx=(0, 8), pady=8)
        self.bin_listbox.configure(yscrollcommand=scrollbar.set)

        selected = ttk.Frame(parent)
        selected.grid(row=2, column=0, sticky='ew', pady=(8, 4))
        selected.columnconfigure(1, weight=1)
        ttk.Label(selected, text='Selected Bin ID').grid(
            row=0, column=0, sticky='w', pady=4)
        ttk.Entry(selected, textvariable=self.selected_bin_var).grid(
            row=0, column=1, sticky='ew', padx=(8, 0), pady=4)

        button_row = ttk.Frame(parent)
        button_row.grid(row=3, column=0, sticky='ew', pady=(0, 8))
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        ttk.Button(
            button_row,
            text='Use Highlighted Bin',
            command=self._use_highlighted_bin,
        ).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(
            button_row,
            text='Queue Mission',
            command=self._queue_mission,
        ).grid(row=0, column=1, sticky='ew', padx=(4, 0))

        mission_status = ttk.LabelFrame(parent, text='Live Mission Status')
        mission_status.grid(row=4, column=0, sticky='ew')
        mission_status.columnconfigure(1, weight=1)
        mission_status.columnconfigure(3, weight=1)
        status_rows = [
            ('Mission ID', self.active_mission_id_var),
            ('Bin ID', self.active_mission_bin_var),
            ('State', self.mission_state_var),
            ('Action', self.mission_action_var),
            ('Progress', self.mission_progress_var),
            ('Error', self.mission_error_var),
        ]
        for row, (label, variable) in enumerate(status_rows):
            grid_column = 0 if row < 3 else 2
            grid_row = row if row < 3 else row - 3
            ttk.Label(mission_status, text=label).grid(
                row=grid_row,
                column=grid_column,
                sticky='w',
                padx=8,
                pady=3,
            )
            ttk.Label(mission_status, textvariable=variable).grid(
                row=grid_row,
                column=grid_column + 1,
                sticky='w',
                padx=(0, 8),
                pady=3,
            )

    def _build_activity_panel(self, parent):
        self.activity_text = tk.Text(
            parent,
            height=18,
            wrap='word',
            relief='flat',
            borderwidth=0,
            highlightthickness=0,
        )
        self.activity_text.grid(row=0, column=0, sticky='nsew', padx=(8, 0), pady=8)
        self.activity_text.configure(state='disabled')

        scrollbar = ttk.Scrollbar(
            parent,
            orient='vertical',
            command=self.activity_text.yview,
        )
        scrollbar.grid(row=0, column=1, sticky='ns', padx=(0, 8), pady=8)
        self.activity_text.configure(yscrollcommand=scrollbar.set)

    def _build_camera_panel(self, parent):
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

        front_frame = ttk.LabelFrame(parent, text='Front Camera')
        front_frame.grid(row=0, column=0, sticky='nsew', padx=(8, 4), pady=8)
        front_frame.columnconfigure(0, weight=1)

        rear_frame = ttk.LabelFrame(parent, text='Rear Camera')
        rear_frame.grid(row=0, column=1, sticky='nsew', padx=(4, 8), pady=8)
        rear_frame.columnconfigure(0, weight=1)

        self.front_camera_canvas = tk.Canvas(
            front_frame,
            width=360,
            height=270,
            bg='black',
            highlightthickness=1,
            highlightbackground='#777777',
        )
        self.front_camera_canvas.grid(row=0, column=0, padx=8, pady=(8, 6))

        self.rear_camera_canvas = tk.Canvas(
            rear_frame,
            width=360,
            height=270,
            bg='black',
            highlightthickness=1,
            highlightbackground='#777777',
        )
        self.rear_camera_canvas.grid(row=0, column=0, padx=8, pady=(8, 6))

        ttk.Label(
            front_frame,
            textvariable=self.front_camera_status_var,
            wraplength=360,
            justify='left',
        ).grid(row=1, column=0, sticky='w', padx=8, pady=(0, 8))

        ttk.Label(
            rear_frame,
            textvariable=self.rear_camera_status_var,
            wraplength=360,
            justify='left',
        ).grid(row=1, column=0, sticky='w', padx=8, pady=(0, 8))

    def _append_log(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.activity_text.configure(state='normal')
        self.activity_text.insert('end', f'[{timestamp}] {message}\n')
        self.activity_text.see('end')
        self.activity_text.configure(state='disabled')

    def _add_section_label(self, parent, title, row):
        label = ttk.Label(parent, text=title, font=('TkDefaultFont', 11, 'bold'))
        label.grid(
            row=row,
            column=0,
            columnspan=2,
            sticky='w',
            pady=(6 if row else 0, 3),
        )

    def _add_button(self, parent, text, key, row, col, colspan=1):
        button = ttk.Button(parent, text=text, command=lambda: self._dispatch_key(key))
        button.grid(
            row=row,
            column=col,
            columnspan=colspan,
            sticky='ew',
            padx=4,
            pady=2,
        )

    def _is_typing_widget_focused(self):
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

    def _on_keypress(self, event):
        if self._is_typing_widget_focused():
            return

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

    def _initial_refresh(self):
        try:
            self._refresh_bins(log_result=False)
        except Exception as exc:
            self._append_log(f'Initial bin refresh failed: {exc}')

    def _backend_request(self, method, path, payload=None):
        base_url = self.backend_url_var.get().strip().rstrip('/')
        if not base_url:
            raise RuntimeError('Backend URL is required.')

        data = None
        headers = {}
        if payload is not None:
            data = json.dumps(payload).encode('utf-8')
            headers['Content-Type'] = 'application/json'

        request = urllib.request.Request(
            f'{base_url}{path}',
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=3.0) as response:
                raw_body = response.read()
        except urllib.error.HTTPError as exc:
            raw_error = exc.read().decode('utf-8', errors='replace')
            try:
                detail = json.loads(raw_error).get('detail', raw_error)
            except ValueError:
                detail = raw_error or str(exc)
            raise RuntimeError(f'Backend error {exc.code}: {detail}') from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'Backend request failed: {exc.reason}') from exc

        if not raw_body:
            return None
        return json.loads(raw_body.decode('utf-8'))

    def _start_path_recording(self):
        path_id = self.path_id_var.get().strip()
        description = self.path_description_var.get().strip()
        success, message = self.node.start_path_recording(path_id, description)
        self.path_status_var.set(message)
        self._append_log(message)

        if success:
            self._last_saved_path_id = ''
            self.last_saved_path_var.set('Recording in progress...')

    def _stop_path_recording(self):
        result = self.node.stop_path_recording()
        self.path_status_var.set(result['message'])
        self._append_log(result['message'])

        if result['success']:
            self._last_saved_path_id = result['path_id']
            self.last_saved_path_var.set(
                f"{result['path_id']} | {result['num_points']} pts | "
                f"{result['total_distance']:.2f} m"
            )
            if not self.path_to_bin_var.get().strip():
                self.path_to_bin_var.set(result['path_id'])

    def _apply_last_path(self, target_var, field_name):
        if not self._last_saved_path_id:
            message = 'No saved path is available yet.'
            self.path_status_var.set(message)
            self._append_log(message)
            return

        target_var.set(self._last_saved_path_id)
        message = f'Applied {self._last_saved_path_id} to {field_name}.'
        self.path_status_var.set(message)
        self._append_log(message)

    def _create_bin(self):
        try:
            pickup_marker_id = int(self.pickup_marker_var.get().strip())
        except ValueError:
            message = 'Pickup marker ID must be an integer.'
            self.bin_status_var.set(message)
            self._append_log(message)
            return

        payload = {
            'rfid_tag_id': self.rfid_tag_var.get().strip(),
            'bin_type': self.bin_type_var.get().strip(),
            'location_name': self.location_name_var.get().strip(),
            'path_to_bin_id': self.path_to_bin_var.get().strip(),
            'path_to_collection_id': self.path_to_collection_var.get().strip(),
            'path_to_home_id': self.path_to_home_var.get().strip(),
            'pickup_marker_id': pickup_marker_id,
            'home_pose_x': 0.0,
            'home_pose_y': 0.0,
            'home_pose_z': 0.0,
            'home_pose_qx': 0.0,
            'home_pose_qy': 0.0,
            'home_pose_qz': 0.0,
            'home_pose_qw': 1.0,
        }

        missing_fields = [
            field_name
            for field_name, value in (
                ('rfid_tag_id', payload['rfid_tag_id']),
                ('bin_type', payload['bin_type']),
                ('location_name', payload['location_name']),
                ('path_to_bin_id', payload['path_to_bin_id']),
                ('path_to_collection_id', payload['path_to_collection_id']),
                ('path_to_home_id', payload['path_to_home_id']),
            )
            if not value
        ]
        if missing_fields:
            message = f'Missing required fields: {", ".join(missing_fields)}'
            self.bin_status_var.set(message)
            self._append_log(message)
            return

        try:
            response = self._backend_request('POST', '/api/bins', payload)
        except RuntimeError as exc:
            self.bin_status_var.set(str(exc))
            self._append_log(str(exc))
            return

        bin_id = response.get('bin_id', '')
        self.selected_bin_var.set(bin_id)
        self.bin_status_var.set(f'Created bin {bin_id}.')
        self._append_log(
            f"Created bin {bin_id} for {payload['location_name']} "
            f"(marker {pickup_marker_id})."
        )
        self._refresh_bins(select_bin_id=bin_id, log_result=False)

    def _refresh_bins(self, select_bin_id=None, log_result=True):
        try:
            response = self._backend_request('GET', '/api/bins?active_only=true')
        except RuntimeError as exc:
            self.bin_status_var.set(str(exc))
            if log_result:
                self._append_log(str(exc))
            return

        self._bins = response or []
        self.bin_listbox.delete(0, tk.END)
        selected_index = None
        for index, bin_info in enumerate(self._bins):
            label = (
                f"{bin_info['bin_id']} | {bin_info['location_name']} | "
                f"{bin_info['bin_type']} | marker {bin_info['pickup_marker_id']}"
            )
            self.bin_listbox.insert(tk.END, label)
            if bin_info['bin_id'] == select_bin_id:
                selected_index = index

        self.bin_status_var.set(f'Loaded {len(self._bins)} active bins.')
        if log_result:
            self._append_log(f'Refreshed bin list ({len(self._bins)} active bins).')

        if selected_index is not None:
            self.bin_listbox.selection_clear(0, tk.END)
            self.bin_listbox.selection_set(selected_index)
            self.bin_listbox.activate(selected_index)
            self._on_bin_selected()

    def _selected_bin_info(self):
        selection = self.bin_listbox.curselection()
        if not selection:
            return None
        index = selection[0]
        if index < 0 or index >= len(self._bins):
            return None
        return self._bins[index]

    def _on_bin_selected(self, event=None):
        del event
        bin_info = self._selected_bin_info()
        if bin_info is None:
            return
        self.selected_bin_var.set(bin_info['bin_id'])
        self._append_log(
            f"Selected {bin_info['bin_id']} for {bin_info['location_name']}."
        )

    def _use_highlighted_bin(self):
        bin_info = self._selected_bin_info()
        if bin_info is None:
            message = 'No bin is highlighted in the list.'
            self.mission_request_var.set(message)
            self._append_log(message)
            return

        self.selected_bin_var.set(bin_info['bin_id'])
        message = f"Ready to queue {bin_info['bin_id']}."
        self.mission_request_var.set(message)
        self._append_log(message)

    def _queue_mission(self):
        bin_id = self.selected_bin_var.get().strip()
        if not bin_id:
            bin_info = self._selected_bin_info()
            if bin_info is not None:
                bin_id = bin_info['bin_id']
                self.selected_bin_var.set(bin_id)

        accepted, mission_id, message = self.node.queue_mission(bin_id)
        self.mission_request_var.set(message)
        self._append_log(message)

        if accepted:
            self.active_mission_id_var.set(mission_id or '-')
            if bin_id:
                self.active_mission_bin_var.set(bin_id)

    def _update_status(self):
        self.motion_var.set(
            f'linear_x={self.node.linear_x:.2f} m/s, '
            f'angular_z={self.node.angular_z:.2f} rad/s'
        )
        self.servo_var.set(
            f'left={self.node.servo_left:.2f}, right={self.node.servo_right:.2f}'
        )
        if self.node.hook_attached is None:
            attached_text = 'unknown'
        else:
            attached_text = 'yes' if self.node.hook_attached else 'no'
        self.hook_var.set(
            f'state={self.node.hook_state}, bin_attached={attached_text}'
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

        mission_status = self.node.latest_mission_status
        self.active_mission_id_var.set(mission_status.get('mission_id') or '-')
        self.active_mission_bin_var.set(mission_status.get('bin_id') or '-')
        self.mission_state_var.set(mission_status.get('state') or 'IDLE')
        self.mission_action_var.set(mission_status.get('current_action') or '-')
        self.mission_progress_var.set(
            f"{mission_status.get('progress_percent', 0.0):.1f}%"
        )
        self.mission_error_var.set(mission_status.get('error_message') or 'None')
        self._update_camera_views()

    def _update_camera_views(self):
        self._render_camera_preview(
            camera_name='front',
            canvas=self.front_camera_canvas,
            status_var=self.front_camera_status_var,
            photo_attr='_front_camera_photo',
            seq_attr='_front_camera_seq',
            fallback_text='Waiting for front camera feed',
        )
        self._render_camera_preview(
            camera_name='rear',
            canvas=self.rear_camera_canvas,
            status_var=self.rear_camera_status_var,
            photo_attr='_rear_camera_photo',
            seq_attr='_rear_camera_seq',
            fallback_text='Waiting for rear camera feed',
        )

    def _render_camera_preview(
        self,
        camera_name,
        canvas,
        status_var,
        photo_attr,
        seq_attr,
        fallback_text,
    ):
        preview = self.node.get_camera_preview(camera_name)
        status_var.set(preview['status'])
        latest_seq = int(preview['seq'])
        if latest_seq == getattr(self, seq_attr):
            return

        setattr(self, seq_attr, latest_seq)
        setattr(self, photo_attr, None)
        canvas.delete('all')

        ppm_data = preview['ppm']
        if ppm_data:
            photo = tk.PhotoImage(data=ppm_data, format='PPM')
            setattr(self, photo_attr, photo)
            canvas.create_image(
                int(canvas.cget('width')) // 2,
                int(canvas.cget('height')) // 2,
                image=photo,
            )
            return

        canvas.create_text(
            int(canvas.cget('width')) // 2,
            int(canvas.cget('height')) // 2,
            text=fallback_text,
            fill='white',
            width=max(100, int(canvas.cget('width')) - 32),
            justify='center',
        )

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
