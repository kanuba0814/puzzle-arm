"""
机械臂控制 GUI (Tkinter, 无第三方依赖, 树莓派直接可跑)

运行:
    python3 gui_app.py            # 自动选择后端 (当前为 Mock 模拟后端)

界面布局:
    顶部状态栏   : 连接/模拟/力矩/电磁铁/标定/任务状态 + 各关节角度
    左侧 自动控制: 一键标定 / 开始检测搬运(task1/task2) / 中止 / 急停 / 进度条
    中间 手动控制: 力矩使能/失能, 电磁铁, 回Home,
                   J1-J5 关节步进(+/−), 末端 XYZ 步进(+/−), 步长选择
    底部 日志区   : 滚动日志 + 清空按钮

架构说明见 gui_backend.py 头部注释: GUI 只依赖 RobotBackend 接口,
所有回调经 queue 转入 Tk 主线程, 后端线程不直接操作控件。
"""
import queue
import tkinter as tk
from tkinter import ttk

from gui_backend import RobotState, create_backend

JOG_STEPS_DEG = [1, 5, 10, 30]     # 关节步进可选步长 (度)
JOG_STEPS_MM = [1, 5, 10, 50]      # 末端步进可选步长 (mm)
POLL_MS = 200                       # 状态轮询周期

STATE_LABELS = {
    'IDLE': '空闲', 'CALIBRATING': '标定中', 'DETECTING': '检测中',
    'SOLVING': '求解中', 'EXECUTING': '执行中', 'DONE': '完成', 'ERROR': '错误',
}


class ArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('拼图机械臂控制台 (SO-ARM101)')
        self.geometry('980x640')
        self.minsize(900, 600)

        self.backend = create_backend()
        self._cb_queue = queue.Queue()  # 后端线程 -> GUI 主线程

        self.backend.set_callbacks(
            on_log=lambda lvl, msg: self._cb_queue.put(('log', lvl, msg)),
            on_state=lambda st: self._cb_queue.put(('state', st)),
            on_progress=lambda stage, f: self._cb_queue.put(('progress', stage, f)),
            on_task_done=lambda ok, msg: self._cb_queue.put(('task_done', ok, msg)),
        )

        self._build_ui()
        ok, msg = self.backend.connect()
        self._log('INFO' if ok else 'ERROR', f'后端连接: {msg}')

        self.after(50, self._drain_callbacks)
        self.protocol('WM_DELETE_WINDOW', self._on_close)

    # ==================== UI 构建 ====================

    def _build_ui(self):
        self._build_status_bar()
        body = ttk.Frame(self, padding=6)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=1, uniform='col')
        body.columnconfigure(1, weight=1, uniform='col')
        body.rowconfigure(0, weight=1)
        self._build_auto_panel(body)
        self._build_manual_panel(body)
        self._build_log_panel()

    def _build_status_bar(self):
        bar = ttk.LabelFrame(self, text='状态', padding=6)
        bar.pack(fill='x', padx=6, pady=(6, 0))
        self._status_vars = {}
        items = [('connected', '连接'), ('simulation', '模式'),
                 ('torque', '力矩'), ('magnet', '电磁铁'),
                 ('calibrated', '标定'), ('task', '任务')]
        for i, (key, label) in enumerate(items):
            ttk.Label(bar, text=f'{label}:').grid(row=0, column=i * 2, padx=(8, 2))
            var = tk.StringVar(value='-')
            self._status_vars[key] = var
            ttk.Label(bar, textvariable=var, font=('TkDefaultFont', 10, 'bold')
                      ).grid(row=0, column=i * 2 + 1)
        # 关节角度行
        self._joint_var = tk.StringVar(value='J: -')
        ttk.Label(bar, textvariable=self._joint_var).grid(
            row=1, column=0, columnspan=12, sticky='w', pady=(4, 0))

    def _build_auto_panel(self, parent):
        f = ttk.LabelFrame(parent, text='自动控制', padding=8)
        f.grid(row=0, column=0, sticky='nsew', padx=(0, 3))

        ttk.Button(f, text='① 一键标定 (相机/单应矩阵)',
                   command=self._on_calibrate).pack(fill='x', pady=4)

        row = ttk.Frame(f)
        row.pack(fill='x', pady=4)
        self._task_mode = tk.StringVar(value='task1')
        ttk.Radiobutton(row, text='任务1 纯色碎片', variable=self._task_mode,
                        value='task1').pack(side='left')
        ttk.Radiobutton(row, text='任务2 扑克牌碎片', variable=self._task_mode,
                        value='task2').pack(side='left', padx=8)

        ttk.Button(f, text='② 开始检测搬运 (自动全流程)',
                   command=self._on_start_task).pack(fill='x', pady=4)
        ttk.Button(f, text='中止任务', command=self._on_abort).pack(fill='x', pady=4)

        stop = tk.Button(f, text='急 停 (失能力矩)', bg='#d32f2f', fg='white',
                         font=('TkDefaultFont', 11, 'bold'),
                         command=self._on_estop)
        stop.pack(fill='x', pady=(12, 4), ipady=6)

        ttk.Label(f, text='进度:').pack(anchor='w', pady=(10, 0))
        self._progress = ttk.Progressbar(f, maximum=100)
        self._progress.pack(fill='x', pady=2)
        self._progress_label = ttk.Label(f, text='-')
        self._progress_label.pack(anchor='w')

    def _build_manual_panel(self, parent):
        f = ttk.LabelFrame(parent, text='手动控制', padding=8)
        f.grid(row=0, column=1, sticky='nsew', padx=(3, 0))

        # --- 使能/电磁铁/Home ---
        row = ttk.Frame(f)
        row.pack(fill='x', pady=2)
        ttk.Button(row, text='力矩使能', width=10,
                   command=lambda: self._torque(True)).pack(side='left', padx=2)
        ttk.Button(row, text='力矩失能', width=10,
                   command=lambda: self._torque(False)).pack(side='left', padx=2)
        ttk.Button(row, text='电磁铁 吸', width=10,
                   command=lambda: self._magnet(True)).pack(side='left', padx=2)
        ttk.Button(row, text='电磁铁 放', width=10,
                   command=lambda: self._magnet(False)).pack(side='left', padx=2)
        ttk.Button(f, text='回初始位置 (Home)',
                   command=self.backend.home).pack(fill='x', pady=4)

        # --- 关节步进 ---
        joint_box = ttk.LabelFrame(f, text='关节步进 (度)', padding=4)
        joint_box.pack(fill='x', pady=6)
        self._joint_step = tk.DoubleVar(value=5)
        step_row = ttk.Frame(joint_box)
        step_row.pack(fill='x')
        ttk.Label(step_row, text='步长:').pack(side='left')
        for s in JOG_STEPS_DEG:
            ttk.Radiobutton(step_row, text=str(s), value=s,
                            variable=self._joint_step).pack(side='left')
        for j in range(5):
            r = ttk.Frame(joint_box)
            r.pack(fill='x', pady=1)
            ttk.Label(r, text=f'J{j+1}', width=4).pack(side='left')
            ttk.Button(r, text='−', width=4,
                       command=lambda j=j: self._jog_joint(j, -1)
                       ).pack(side='left', padx=2)
            ttk.Button(r, text='+', width=4,
                       command=lambda j=j: self._jog_joint(j, +1)
                       ).pack(side='left', padx=2)
            var = tk.StringVar(value='0.0°')
            setattr(self, f'_jvar{j}', var)
            ttk.Label(r, textvariable=var).pack(side='left', padx=8)

        # --- 末端 XYZ 步进 ---
        xyz_box = ttk.LabelFrame(f, text='末端步进 (mm)', padding=4)
        xyz_box.pack(fill='x', pady=6)
        self._xyz_step = tk.DoubleVar(value=10)
        step_row2 = ttk.Frame(xyz_box)
        step_row2.pack(fill='x')
        ttk.Label(step_row2, text='步长:').pack(side='left')
        for s in JOG_STEPS_MM:
            ttk.Radiobutton(step_row2, text=str(s), value=s,
                            variable=self._xyz_step).pack(side='left')
        grid = ttk.Frame(xyz_box)
        grid.pack(pady=4)
        moves = [('X−', 'x', -1), ('X+', 'x', +1),
                 ('Y−', 'y', -1), ('Y+', 'y', +1),
                 ('Z−', 'z', -1), ('Z+', 'z', +1)]
        for i, (txt, axis, sign) in enumerate(moves):
            ttk.Button(grid, text=txt, width=6,
                       command=lambda a=axis, s=sign: self._jog_xyz(a, s)
                       ).grid(row=i // 2, column=i % 2, padx=3, pady=2)

    def _build_log_panel(self):
        f = ttk.LabelFrame(self, text='日志', padding=4)
        f.pack(fill='both', expand=True, padx=6, pady=6)
        self._log_text = tk.Text(f, height=10, state='disabled', wrap='none')
        sb = ttk.Scrollbar(f, command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=sb.set)
        self._log_text.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        ttk.Button(f, text='清空', command=self._clear_log).pack(side='right', anchor='n')
        for tag, color in [('INFO', '#222'), ('WARN', '#b26a00'), ('ERROR', '#c00')]:
            self._log_text.tag_configure(tag, foreground=color)

    # ==================== 动作 ====================

    def _torque(self, enable):
        ok, msg = self.backend.set_torque(enable)
        if not ok:
            self._log('ERROR', msg)

    def _magnet(self, on):
        self.backend.set_magnet(on)

    def _jog_joint(self, joint_idx, sign):
        delta = sign * self._joint_step.get()
        ok, msg = self.backend.jog_joint(joint_idx, delta)
        if not ok:
            self._log('ERROR', msg)

    def _jog_xyz(self, axis, sign):
        d = sign * self._xyz_step.get()
        kw = {'x': (d, 0, 0), 'y': (0, d, 0), 'z': (0, 0, d)}[axis]
        self.backend.jog_xyz(*kw)

    def _on_calibrate(self):
        self._progress['value'] = 0
        self.backend.calibrate()

    def _on_start_task(self):
        ok, msg = self.backend.start_task(self._task_mode.get())
        self._log('INFO' if ok else 'WARN', msg)

    def _on_abort(self):
        self.backend.abort()

    def _on_estop(self):
        self.backend.emergency_stop()

    # ==================== 回调处理 (主线程) ====================

    def _drain_callbacks(self):
        try:
            while True:
                item = self._cb_queue.get_nowait()
                kind = item[0]
                if kind == 'log':
                    self._log(item[1], item[2])
                elif kind == 'state':
                    self._render_state(item[1])
                elif kind == 'progress':
                    self._progress['value'] = item[2] * 100
                    self._progress_label.config(
                        text=f'{item[1]} {item[2]*100:.0f}%')
                elif kind == 'task_done':
                    ok, msg = item[1], item[2]
                    self._log('INFO' if ok else 'WARN', f'任务结束: {msg}')
        except queue.Empty:
            pass
        self.after(50, self._drain_callbacks)

    def _render_state(self, st: RobotState):
        v = self._status_vars
        v['connected'].set('已连接' if st.connected else '未连接')
        v['simulation'].set('模拟' if st.simulation else '硬件')
        v['torque'].set('使能' if st.torque_enabled else '失能')
        v['magnet'].set('吸合' if st.magnet_on else '释放')
        v['calibrated'].set('已标定' if st.calibrated else '未标定')
        v['task'].set(STATE_LABELS.get(st.task_state, st.task_state))
        for j, deg in enumerate(st.joints_deg):
            getattr(self, f'_jvar{j}').set(f'{deg:.1f}°')
        self._joint_var.set('关节角: ' + '  '.join(
            f'J{j+1}={d:.1f}°' for j, d in enumerate(st.joints_deg)))

    def _log(self, level, msg):
        self._log_text.configure(state='normal')
        self._log_text.insert('end', f'[{level}] {msg}\n', level)
        self._log_text.see('end')
        self._log_text.configure(state='disabled')

    def _clear_log(self):
        self._log_text.configure(state='normal')
        self._log_text.delete('1.0', 'end')
        self._log_text.configure(state='disabled')

    def _on_close(self):
        self.backend.disconnect()
        self.destroy()


if __name__ == '__main__':
    app = ArmGUI()
    app.mainloop()
