import tkinter as tk
import threading
import numpy as np
import pickle
from bayes_opt import BayesianOptimization as BO
import datetime
import os
import rospkg
import shutil

normal_font = ('Arial', '10')
title_font = ('Arial', '9', 'bold')
small_font = ('Arial', '9')


class LearningGUI:
    def __init__(self, learning_client):
        self.learning_client = learning_client

        self.parent = tk.Tk()
        self.parent.resizable(0, 0)
        self.parent.minsize(500, 500)
        self.parent.maxsize(1000, 1000)
        self.parent.title('Parameters Learning GUI')
        self.parent.bind('<Control-c>', lambda e: self.parent.destroy())

        self.task_ctr_frame = tk.LabelFrame(self.parent, font=title_font, text="Task Settings")
        self.lrning_frame = tk.LabelFrame(self.parent, font=title_font, text="Bayesian Optimization Settings and Info")
        self.dyn_config_frame = tk.LabelFrame(self.parent, height=400, font=title_font, text="Dynamic Reconfigure Settings")

        self.lrning_frame.grid(row=0, column=1, sticky='wens', padx=2, pady=2)
        self.task_ctr_frame.grid(row=0, column=0, sticky='wns', padx=2, pady=2)
        self.dyn_config_frame.grid(row=1, column=0, columnspan=2, sticky='ns', padx=2, pady=2)
        self.parent.rowconfigure(1, minsize=400)
        self.parent.columnconfigure(1, weight=1)

        # --------- Task Control Frame --------- #
        self.task_ctr=dict()

        self.task_ctr['start'] = tk.Button(self.task_ctr_frame, width=10, font=normal_font, text='Start Task', command=self.start_task_once)
        self.task_ctr['fail'] = tk.Button(self.task_ctr_frame, width=10, font=normal_font, text='FAILURE', command=self.learning_client.cancel_task, fg='red')

        task_type_frame = tk.Frame(self.task_ctr_frame)
        v = tk.IntVar(task_type_frame)
        v.set(0)
        self.learning_client.set_action_settings(atype=0)
        self.task_ctr['circle'] = tk.Radiobutton(task_type_frame, text="Circle", variable=v, value=0,
                       command=lambda: self.learning_client.set_action_settings(atype=0))
        self.task_ctr['square'] = tk.Radiobutton(task_type_frame, text="Square", variable=v, value=1,
                       command=lambda: self.learning_client.set_action_settings(atype=1))
        self.task_ctr['circle'].pack(side='left', anchor='center')
        self.task_ctr['square'].pack(side='left', anchor='center')

        self.task_ctr['depth'] = tk.Scale(self.task_ctr_frame, from_=0.0, to=1.0, resolution=0.01,
                                    orient='horizontal', font=small_font, tickinterval=1, length=250,
                                    command=lambda x: self.learning_client.set_action_settings(depth=float(x)))
        self.task_ctr['radius'] = tk.Scale(self.task_ctr_frame, from_=0.1, to=1.5, resolution=0.1,
                                     orient='horizontal', font=small_font, tickinterval=1.4, length=250,
                                     command=lambda x: self.learning_client.set_action_settings(radius=float(x)))
        self.task_ctr['speed'] = tk.Scale(self.task_ctr_frame, from_=0.1, to=5.0, resolution=0.1,
                                    orient='horizontal', font=small_font, tickinterval=4.9, length=250,
                                    command=lambda x: self.learning_client.set_action_settings(speed=float(x)))

        self.task_ctr['depth'].set(0.05)
        self.task_ctr['radius'].set(1)
        self.task_ctr['speed'].set(0.2)
        self.task_ctr['start'].grid(columnspan=2, sticky='we', padx=2, pady=2)
        self.task_ctr['fail'].grid(row=0, column=2, columnspan=2, sticky='we', padx=2, pady=2)
        self.create_labels(self.task_ctr_frame, 1, 0, 0, 'Type', 'Depth', 'Radius', 'Speed')
        task_type_frame.grid(row=1, column=1, columnspan=3, sticky='we', padx=2, pady=2)
        self.task_ctr['depth'].grid(row=2, column=1, columnspan=3, sticky='we', padx=2, pady=2)
        self.task_ctr['radius'].grid(row=3, column=1, columnspan=3, sticky='we',  padx=2, pady=2)
        self.task_ctr['speed'].grid(row=4, column=1, columnspan=3, sticky='we', padx=2, pady=2)

        # --------- Bayesian Optimization and Info Frame --------- #
        self.lrning_ctr = dict()
        self.lrning_ctr['init'] = tk.Button(self.lrning_frame, text='Add Initial Point', font=normal_font,
                                            command=self.add_last_init_pt)
        self.lrning_ctr['raninit'] = tk.Button(self.lrning_frame, text='Add Random Initial Points',
                                                   font=normal_font, command=self.add_rand_init_pt)
        self.lrning_ctr['points'] = tk.Scale(self.lrning_frame, from_=1, to=10, orient='horizontal',
                                             font=small_font, tickinterval=9, length=150)
        self.lrning_ctr['start'] = tk.Button(self.lrning_frame, text='Start Learning', font=normal_font, command=self.learn)
        self.lrning_ctr['iter'] = tk.Scale(self.lrning_frame, from_=1, to=40, orient='horizontal',
                                           font=small_font, tickinterval=39, length=150)
        self.lrning_ctr['cancel'] = tk.Button(self.lrning_frame, text='Cancel Learning', width=12, font=normal_font,
                                           command=self.reset_learning, fg='red')
        self.lrning_ctr['load'] = tk.Button(self.lrning_frame, text='Automatic Load & Learn', font=normal_font,
                                            command=self.load_init_pts)
        self.lrning_ctr['save'] = tk.Button(self.lrning_frame, text='Save Results', font=normal_font,
                                         command=self.save_results)

        self.lrning_ctr['init'].grid(row=0, column=0, columnspan=2, sticky='we', padx=2, pady=2)
        self.lrning_ctr['raninit'].grid(row=1, column=0, columnspan=2, sticky='we', padx=2, pady=2)
        tk.Label(self.lrning_frame, font=small_font, anchor='e', width=8, text='Points').grid(row=2, column=0, padx=2, pady=2)
        self.lrning_ctr['points'].grid(row=2, column=1, sticky='we', padx=2, pady=2)
        self.lrning_ctr['start'].grid(row=3, column=0, columnspan=2, sticky='we', padx=2, pady=2)
        tk.Label(self.lrning_frame, font=small_font, anchor='e', width=8, text='Iterations').grid(row=4, column=0, padx=2, pady=2)
        self.lrning_ctr['iter'].grid(row=4, column=1, sticky='we', padx=2, pady=2)
        self.lrning_ctr['save'].grid(row=0, column=2, sticky='we', padx=2, pady=2)
        self.lrning_ctr['load'].grid(row=0, column=3, sticky='we', padx=2, pady=2)
        self.lrning_ctr['cancel'].grid(row=1, column=2, columnspan=2, sticky='we', padx=2, pady=2)


        self.params_frame = tk.LabelFrame(self.lrning_frame, font=title_font, text="Learning Params.")
        self.params_frame.grid(row=2, rowspan=3, column=2, sticky='ns', padx=2, pady=2)
        self.info_frame = tk.LabelFrame(self.lrning_frame, font=title_font, text="Information")
        self.info_frame.grid(row=2, rowspan=3, column=3, sticky='wens', padx=2, pady=2)
        self.lrning_frame.columnconfigure(3, weight=1)


        self.create_labels(self.params_frame, 0, 0, 0, 'Alpha', 'xi/kappa', 'Acq. Fun.', 'Loc. Opt.', 'd', 'logistic', anchor='e', width=8)
        self.lrning_param = dict()
        self.lrning_param['alpha'] = tk.Entry(self.params_frame, font=small_font, width=10)
        self.lrning_param['alpha'].insert(0, '1e-10')
        self.lrning_param['const'] = tk.Entry(self.params_frame, font=small_font, width=10)
        self.lrning_param['const'].insert(0, '0.001')
        self.lrning_param['var'] = tk.StringVar(self.params_frame)
        self.lrning_param['var'].set('poi')
        self.lrning_param['acq'] = tk.OptionMenu(self.params_frame, self.lrning_param['var'], 'poi', 'ei', 'ucb')
        self.lrning_param['acq'].config(font=small_font)

        self.lrning_param['locvar'] = tk.IntVar()
        self.lrning_param['local'] = tk.Checkbutton(self.params_frame, variable=self.lrning_param['locvar'])
        self.lrning_param['dist'] = tk.Entry(self.params_frame, font=small_font, width=10)
        self.lrning_param['dist'].insert(0, '0.1')
        self.lrning_param['logvar'] = tk.IntVar()
        self.lrning_param['logis'] = tk.Checkbutton(self.params_frame, variable=self.lrning_param['logvar'])

        self.lrning_param['alpha'].grid(row=0, column=1, padx=2, pady=2)
        self.lrning_param['const'].grid(row=1, column=1, padx=2, pady=2)
        self.lrning_param['acq'].grid(row=2, column=1, sticky='we', padx=2, pady=2)
        self.lrning_param['local'].grid(row=3, column= 1, sticky='we', padx=2, pady=2)
        self.lrning_param['dist'].grid(row=4, column=1, padx=2, pady=2)
        self.lrning_param['logis'].grid(row=5, column= 1, sticky='we', padx=2, pady=2)



        self.create_labels(self.info_frame, 0, 0, 0, 'Last Reward', 'Max Reward', 'Initial Points',
                           'Iterations', 'Dimensions', anchor='e', width=11)
        self.lrning_info = dict()
        self.lrning_info['last'] = tk.Label(self.info_frame, text='---', font=small_font, anchor='w', width=11, bd=2, relief='groove')
        self.lrning_info['max'] = tk.Label(self.info_frame, text='---', font=small_font, anchor='w', width=11, bd=2, relief='groove')
        self.lrning_info['init'] = tk.Label(self.info_frame, text='0', font=small_font, anchor='w', width=11, bd=2, relief='groove')
        self.lrning_info['itrs'] = tk.Label(self.info_frame, text='0', font=small_font, anchor='w', width=11, bd=2, relief='groove')
        self.lrning_info['dim'] = tk.Label(self.info_frame, text='0', font=small_font, anchor='w', width=11, bd=2, relief='groove')

        self.lrning_info['last'].grid(row=0, column=1, padx=2, pady=2)
        self.lrning_info['max'].grid(row=1, column=1, padx=2, pady=2)
        self.lrning_info['init'].grid(row=2, column=1, padx=2, pady=2)
        self.lrning_info['itrs'].grid(row=3, column=1, padx=2, pady=2)
        self.lrning_info['dim'].grid(row=4, column=1, padx=2, pady=2)

        # --------- Dynamic Reconfiguration Frame --------- #
        self.ctr_vars_frame = tk.LabelFrame(self.dyn_config_frame, height=400, font=title_font,
                                            text="Controlled Variables")
        self.fix_vars_frame = tk.LabelFrame(self.dyn_config_frame, height=400, font=title_font,
                                            text="Fixed Variables")
        self.link_vars_frame = tk.LabelFrame(self.dyn_config_frame, height=400, font=title_font,
                                             text="Linked Variables")
        self.dyn_ctr_frame = tk.Frame(self.dyn_config_frame)

        self.dyn_ctr_frame.pack(side='top', fill='x')
        self.ctr_vars_frame.pack(side='left', fill='y', padx=2, pady=2)
        self.fix_vars_frame.pack(side='left', fill='y', padx=2, pady=2)
        self.link_vars_frame.pack(side='left', fill='y', padx=2, pady=2)

        # --------- Dynamic Reconfiguration Control Frame --------- #
        self.dyn_update_button = tk.Button(self.dyn_ctr_frame, text='Send Values', width=12, font=normal_font,
                                           command=self.send_dyn_reconfig_vals)
        self.dyn_show_button = tk.Button(self.dyn_ctr_frame, text='Show Current', width=12, font=normal_font,
                                         command=self.show_current_dyn_reconfig)
        self.dyn_default_button = tk.Button(self.dyn_ctr_frame, text='Restore Default', width=12, font=normal_font,
                                            command=self.restore_dyn_reconfig_default)
        self.dyn_setmax_button = tk.Button(self.dyn_ctr_frame, text='Set on Max', width=12, font=normal_font,
                                            command=self.set_dyn_reconfig_max)

        self.dyn_update_button.pack(side='left', fill='y', padx=2, pady=2)
        self.dyn_show_button.pack(side='left', fill='y', padx=2, pady=2)
        self.dyn_default_button.pack(side='left', fill='y', padx=2, pady=2)
        self.dyn_setmax_button.pack(side='left', fill='y', padx=2, pady=2)

        # ---- Controlled Variables Frame ---- #
        self.create_labels(self.ctr_vars_frame, 0, 0, 1, 'Selected Variables', 'min.', 'set', 'max.')
        self.selected_ctr_vars = []
        self.dyn_ctr_bottom = dict()
        self.dyn_ctr_bottom['var'] = tk.StringVar(self.ctr_vars_frame)
        self.dyn_ctr_bottom['opt'] = tk.OptionMenu(self.ctr_vars_frame, self.dyn_ctr_bottom['var'], 'a')
        self.dyn_ctr_bottom['min'] = tk.Entry(self.ctr_vars_frame, font=small_font, width=5)
        self.dyn_ctr_bottom['val'] = tk.Entry(self.ctr_vars_frame, font=small_font, width=5)
        self.dyn_ctr_bottom['max'] = tk.Entry(self.ctr_vars_frame, font=small_font, width=5)
        self.dyn_ctr_bottom['add'] = tk.Button(self.ctr_vars_frame, font=small_font, text='Add', command=self.add_dyn_ctr_var)

        # ---- Fixed Variables Frame ---- #
        self.create_labels(self.fix_vars_frame, 0, 0, 1, 'Selected Variables', 'set')
        self.selected_fix_vars = []
        self.dyn_fix_bottom = dict()
        self.dyn_fix_bottom['var'] = tk.StringVar(self.fix_vars_frame)
        self.dyn_fix_bottom['opt'] = tk.OptionMenu(self.fix_vars_frame, self.dyn_fix_bottom['var'], 'a')
        self.dyn_fix_bottom['val'] = tk.Entry(self.fix_vars_frame, font=small_font, width=5)
        self.dyn_fix_bottom['add'] = tk.Button(self.fix_vars_frame, font=small_font, text='Add', command=self.add_dyn_fix_var)

        # ---- Linked Variables Frame ---- #
        self.create_labels(self.link_vars_frame, 0, 0, 1, 'Selected Variables', 'Linked')
        self.selected_link_vars = []
        self.dyn_link_bottom = dict()
        self.dyn_link_bottom['var1'] = tk.StringVar(self.link_vars_frame)
        self.dyn_link_bottom['opt1'] = tk.OptionMenu(self.link_vars_frame, self.dyn_link_bottom['var1'], 'a')
        self.dyn_link_bottom['var2'] = tk.StringVar(self.link_vars_frame)
        self.dyn_link_bottom['opt2'] = tk.OptionMenu(self.link_vars_frame, self.dyn_link_bottom['var2'], 'a')
        self.dyn_link_bottom['add'] = tk.Button(self.link_vars_frame, font=small_font, text='Add', command=self.add_dyn_link_var)

        self.reset_learning()
        self.update_bottoms()

        tk.mainloop()

    def create_labels(self, parent, row, column, h, *labels, **config):
        for i in range(len(labels)):
            label = tk.Label(parent, text=labels[i], font=small_font, **config)
            if h:
                label.grid(row=row, column=column+i, padx=2, pady=2)
            else:
                label.grid(row=row+i, column=column, padx=2, pady=2)

    def get_float_entry(self, entry):
        try:
            return float(entry.get())
        except ValueError:
            return None

    def reset_learning(self):
        self.learning_client.cancel_task
        self.bo = None  # Bayesian Optimization class place holder
        self.learning_started = self.task_running = self.learning_initialized = False
        self.iters_running = 0
        self.init_pts_num = 0
        self.last_reward = None
        self.pos_error = []
        self.rot_error = []
        self.toggle_task_ctr()
        self.toggle_lrning_ctr()
        self.toggle_dyn_reconfig_ctr()
        self.update_info()

    def init_bo(self):
        if not self.bo:
            self.bo = BO(self.start_task, self.learning_client.controlled_vars)
            self.learning_started = True
            self.toggle_task_ctr()
            self.toggle_dyn_reconfig_ctr()

    def update_info(self):
        self.lrning_info['dim'].config(text=str(len(self.selected_ctr_vars)))
        if self.bo:
            self.lrning_info['init'].config(text=str(self.init_pts_num))
            if self.bo.space.Y is not None:
                self.lrning_info['max'].config(text=str(max(self.bo.space.Y)))
            else:
                self.lrning_info['max'].config(text=str(max(self.bo.y_init)))
            if self.learning_initialized:
                self.lrning_info['itrs'].config(text=str(len(self.bo.space.Y) - self.init_pts_num))
            else:
                self.lrning_info['itrs'].config(text='0')
        if self.last_reward is not None:
            self.lrning_info['last'].config(text=str(self.last_reward))

    def toggle_task_ctr(self):
        self.task_ctr['start']['state'] = 'disabled' if self.task_running or self.iters_running else 'normal'
        self.task_ctr['fail']['state'] = 'normal' if self.task_running else 'disabled'
        for key in ['depth', 'radius', 'speed', 'circle', 'square']:
            if self.task_running or self.learning_started:
                self.task_ctr[key]['state'] = 'disabled'
            else:
                self.task_ctr[key]['state'] = 'normal'

    def toggle_lrning_ctr(self):
        for key in ['init', 'raninit', 'points']:
            if (not self.task_running) and len(self.selected_ctr_vars) and not self.learning_initialized:
                self.lrning_ctr[key]['state'] = 'normal'
            else:
                self.lrning_ctr[key]['state'] = 'disabled'
        for key in ['start', 'iter']:
            if not (self.task_running or self.iters_running) and self.init_pts_num:
                self.lrning_ctr[key].config(state='normal')
            else:
                self.lrning_ctr[key].config(state='disabled')
        self.lrning_ctr['cancel']['state'] = 'normal' if self.learning_started else 'disabled'
        self.lrning_ctr['save']['state'] = 'normal' if self.learning_initialized and \
                                                       not (self.task_running or self.iters_running) else 'disabled'
        for key in ['alpha', 'const', 'acq', 'local', 'dist']:
            if not (self.task_running or self.iters_running) and self.init_pts_num:
                self.lrning_param[key].config(state='normal')
            else:
                self.lrning_param[key].config(state='disabled')
        self.lrning_param['logis']['state'] = 'normal' if self.learning_started and \
            not (self.task_running or self.iters_running or self.learning_initialized) else 'disabled'

    def toggle_dyn_reconfig_ctr(self):
        if self.task_running or self.learning_started:
            self.dyn_ctr_bottom['add']['state'] = 'disabled'
            self.dyn_fix_bottom['add']['state'] = 'disabled'
            self.dyn_link_bottom['add']['state'] = 'disabled'
            for var, items in (self.selected_ctr_vars + self.selected_fix_vars + self.selected_link_vars):
                items['del']['state'] = 'disabled'
            for var, items in self.selected_fix_vars:
                items['val']['state'] = 'disabled'
        else:
            self.dyn_ctr_bottom['add']['state'] = 'normal'
            self.dyn_fix_bottom['add']['state'] = 'normal'
            self.dyn_link_bottom['add']['state'] = 'normal'
            for var, items in (self.selected_ctr_vars + self.selected_fix_vars + self.selected_link_vars):
                items['del']['state'] = 'normal'
            for var, items in self.selected_fix_vars:
                items['val']['state'] = 'normal'
        if self.task_running or self.iters_running:
            self.dyn_update_button['state'] = 'disabled'
            self.dyn_show_button['state'] = 'disabled'
            self.dyn_default_button['state'] = 'disabled'
        else:
            self.dyn_update_button['state'] = 'normal'
            self.dyn_show_button['state'] = 'normal'
            self.dyn_default_button['state'] = 'normal'
        if self.learning_initialized and not (self.task_running or self.iters_running):
            self.dyn_setmax_button['state'] = 'normal'
        else:
            self.dyn_setmax_button['state'] = 'disabled'

    def start_task(self, **ctr_vars):
        self.task_running = True
        self.toggle_task_ctr()
        self.toggle_dyn_reconfig_ctr()
        self.toggle_lrning_ctr()
        if ctr_vars:
            for var in ctr_vars:
                self.learning_client.set_dyn_reconfig_var(var, ctr_vars[var])
            self.parent.after(200, self.check_new_dyn_vals)
        reward, self.last_pos_error, self.last_rot_error = self.learning_client.start_task()
        self.last_reward = self.convert_reward(reward)
        self.task_running = False
        if self.iters_running:
            self.iters_running -= 1
            if not self.iters_running:
                self.restore_dyn_reconfig_default()  # restore to default after finishing iterations.
            if not self.learning_initialized:
                self.init_pts_num += 1
            self.pos_error.append(self.last_pos_error)
            self.rot_error.append(self.last_rot_error)
        self.toggle_task_ctr()
        self.toggle_dyn_reconfig_ctr()
        self.toggle_lrning_ctr()
        self.parent.after(50, self.update_info)  # Needs to be done after a lag so that self.bo is updated after the current start_task return
        return self.last_reward

    def convert_reward(self, reward):
        if self.learning_initialized and self.enable_sig:
            reward = 1 / (1 + np.exp(-0.5 * (reward - self.sig_mid)))  # logistic function
        return reward

    def check_new_dyn_vals(self):
        if self.learning_client.task_started:
            self.parent.after(100, self.show_current_dyn_reconfig)
        else:
            self.parent.after(200, self.check_new_dyn_vals)

    def start_task_once(self):
        self.task_thread = threading.Thread(target=self.start_task)
        self.task_thread.daemon = True
        self.task_thread.start()

    def add_last_init_pt(self):
        self.init_bo()
        if self.last_reward is not None:
            point = {'target': [self.last_reward]}
            for var in self.learning_client.controlled_vars:
                point[var] = [self.learning_client.current_config[var]]
            try:
                self.bo.initialize(point)
                self.init_pts_num += 1
                self.pos_error.append(self.last_pos_error)
                self.rot_error.append(self.last_rot_error)
                self.toggle_lrning_ctr()
                self.update_info()
            except KeyError:
                print "CANNOT add the same point twice"

    def load_init_pts(self):
        def simulate(init, dim, coef, atype, speed, sig, local):
            with open(str(dim) + 'd' + str(init) + '.pkl', 'rb') as f:
                data = pickle.load(f)
            self.learning_client.controlled_vars = data['controlled_vars']
            self.learning_client.fixed_vars = data['fixed_vars']
            self.learning_client.linked_vars = data['linked_vars']
            print data['controlled_vars']
            print data['fixed_vars']
            print data['linked_vars']

            self.reset_learning()
            self.learning_client.set_action_settings(atype=atype)
            self.learning_client.set_action_settings(speed=speed)
            self.enable_sig = sig

            for i in range(data['init_pts_num']):
                vars = dict(zip(data['keys'], data['x'][i]))
                self.start_task(**vars)
                self.add_last_init_pt()

            self.initialize_learning()
            self.iters_running = 40
            self.bo.maximize(init_points=0,
                             n_iter=40, acq='poi', kappa=0.01, xi=0.01, local_acq_opt=local, d=0.05, alpha=1e-4)
            self.save_results(('square' if atype else 'circle') + str(dim) + 'd' + str(coef) + '-' + str(init)*(2-sig) + 2*str(init)*(not local) + '-2' + 's' + str(speed))

        def sims():
            simulate(1, 4, 1.16, 0, 0.5, True, False)




        self.task_thread = threading.Thread(target=sims)
        self.task_thread.daemon = True
        self.task_thread.start()

    def add_rand_init_pt(self):
        self.init_bo()
        itrs = int(self.lrning_ctr['points'].get())
        self.iters_running = itrs
        self.task_thread = threading.Thread(target=lambda: self.bo.init(itrs))
        self.task_thread.daemon = True
        self.task_thread.start()

    def initialize_learning(self):
        if self.init_pts_num:
            if not self.learning_initialized:
                self.learning_initialized = True
                if not self.bo.initialized:
                    self.bo.init(0)
                self.enable_sig = self.lrning_param['logvar'].get()
                self.sig_mid = max(self.bo.space.Y)
                for i in range(len(self.bo.space.Y)):
                    self.bo.space.Y[i] = self.convert_reward(self.bo.space.Y[i])
                print self.bo.space.Y

    def learn(self):
        self.initialize_learning()
        if self.learning_initialized:
            itrs = int(self.lrning_ctr['iter'].get())
            alpha = self.get_float_entry(self.lrning_param['alpha'])
            const = self.get_float_entry(self.lrning_param['const'])
            d = self.get_float_entry(self.lrning_param['dist'])
            acq = self.lrning_param['var'].get()
            local = self.lrning_param['locvar'].get()
            self.iters_running = itrs
            self.task_thread = threading.Thread(target=lambda: self.bo.maximize(init_points=0,
                                           n_iter=itrs, acq=acq, kappa=const, xi=const, local_acq_opt=local, d=d, alpha=alpha))
            self.task_thread.daemon = True
            self.task_thread.start()

    def save_results(self, filename='learning_results_'):
        data = dict()
        data['x'] = self.bo.space.X
        data['y'] = self.bo.space.Y
        data['keys'] = self.bo.space.keys
        data['pos_error'] = np.vstack(self.pos_error)
        data['rot_error'] = np.vstack(self.rot_error)
        data['action_settings'] = self.learning_client.action_settings
        data['init_pts_num'] = self.init_pts_num
        data['learning_settings'] = {'acq': self.lrning_param['var'].get(),
                                     'loc': self.lrning_param['locvar'].get(),
                                     'd': self.get_float_entry(self.lrning_param['dist']),
                                     'alpha': self.get_float_entry(self.lrning_param['alpha']),
                                     'const': self.get_float_entry(self.lrning_param['const'])}
        data['controlled_vars'] = self.learning_client.controlled_vars
        data['fixed_vars'] = self.learning_client.fixed_vars
        data['linked_vars'] = self.learning_client.linked_vars
        data['sig_mid'] = self.sig_mid

        # Modified to alleviate file overriding problem (Ramy)
        modFileName = filename + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + '.pkl'
        with open(modFileName, 'wb') as f:
            pickle.dump(data, f)

        del data
        shutil.move(os.getcwd() + "/" + modFileName, rospkg.RosPack().get_path('spc_uav_comm') + "/scripts/practical/" + modFileName)
        print "Results Saved !"

    # ---------------- Dynamic Reconfigure Functions --------------- #

    def add_dyn_ctr_var(self):
        var = self.dyn_ctr_bottom['var'].get()
        try:
            min_val = float(self.dyn_ctr_bottom['min'].get())
            max_val = float(self.dyn_ctr_bottom['max'].get())
            if self.learning_client.add_dyn_ctr_var(var, min_val, max_val):
                self.learning_client.set_dyn_reconfig_var(var, self.get_float_entry(self.dyn_ctr_bottom['val']))
                self.add_dyn_ctr_row(var, min_val, max_val)
                self.update_info()
                self.toggle_lrning_ctr()
        except ValueError:
            return  # TODO: Warn the user for invalid input

    def add_dyn_fix_var(self):
        var = self.dyn_fix_bottom['var'].get()
        if self.learning_client.add_dyn_fix_var(var):
            self.learning_client.set_dyn_reconfig_var(var, self.get_float_entry(self.dyn_fix_bottom['val']))
            self.add_dyn_fix_row(var)

    def add_dyn_link_var(self):
        var1 = self.dyn_link_bottom['var1'].get()
        var2 = self.dyn_link_bottom['var2'].get()
        if self.learning_client.add_dyn_link_var(var1, var2):
            self.add_dyn_link_row(var1, var2)

    def add_dyn_ctr_row(self, var, min_val, max_val):
        items = dict()
        items['var'] = tk.Label(self.ctr_vars_frame, font=small_font, text=var, anchor='e')
        items['min'] = tk.Label(self.ctr_vars_frame, font=small_font, text=min_val)
        items['val'] = tk.Entry(self.ctr_vars_frame, font=small_font, width=5)
        items['val'].insert(0, str(self.learning_client.current_config[var]))
        items['max'] = tk.Label(self.ctr_vars_frame, font=small_font, text=max_val)
        items['del'] = tk.Button(self.ctr_vars_frame, font=small_font, text='x', bd=0, relief='flat',
                                 command=lambda: self.delete_dyn_var(var))
        self.selected_ctr_vars.append((var, items))
        self.arrange_columns(items, 'var', 'min', 'val', 'max', 'del')
        self.place_dyn_row(items, len(self.selected_ctr_vars))
        self.update_bottoms()

    def add_dyn_fix_row(self, var):
        items = dict()
        items['var'] = tk.Label(self.fix_vars_frame, font=small_font, text=var, anchor='e')
        items['val'] = tk.Entry(self.fix_vars_frame, font=small_font, width=5)
        items['val'].insert(0, str(self.learning_client.current_config[var]))
        items['del'] = tk.Button(self.fix_vars_frame, font=small_font, text='x', bd=0, relief='flat',
                                 command=lambda: self.delete_dyn_var(var))
        self.selected_fix_vars.append((var, items))
        self.arrange_columns(items, 'var', 'val', 'del')
        self.place_dyn_row(items, len(self.selected_fix_vars))
        self.update_bottoms()

    def add_dyn_link_row(self, var1, var2):
        items = dict()
        items['var1'] = tk.Label(self.link_vars_frame, font=small_font, text=var1, anchor='e')
        items['var2'] = tk.Label(self.link_vars_frame, font=small_font, text='= '+var2, anchor='e')
        items['del'] = tk.Button(self.link_vars_frame, font=small_font, text='x', bd=0, relief='flat',
                                 command=lambda: self.delete_dyn_var(var1))
        self.selected_link_vars.append((var1, items))
        self.arrange_columns(items, 'var1', 'var2', 'del')
        self.place_dyn_row(items, len(self.selected_link_vars))
        self.update_bottoms()

    def arrange_columns(self, items, *columns):
        for i in range(len(columns)):
            config = items[columns[i]].grid_info()
            config['column'] = i
            items[columns[i]].grid(config)

    def place_dyn_row(self, items, row):
        for item in items.values():
            config = item.grid_info()
            config['row'] = row
            item.grid(config)

    def delete_dyn_var(self, var):
        self.learning_client.delete_dyn_reconfig_var(var)
        self.update_vars_rows(self.selected_ctr_vars)
        self.update_vars_rows(self.selected_fix_vars)
        self.update_vars_rows(self.selected_link_vars)
        self.update_bottoms()
        self.update_info()
        self.toggle_lrning_ctr()

    def update_vars_rows(self, selected_vars):
        i = 0
        while i < len(selected_vars):
            var, items = selected_vars[i]
            if var in self.learning_client.get_selectable_vars():
                for item in items.values():
                    item.destroy()
                selected_vars.pop(i)
            else:
                i += 1
        for i in range(len(selected_vars)):
            self.place_dyn_row(selected_vars[i][1], i + 1)

    def update_bottoms(self):
        self.place_dyn_ctr_bottom()
        self.place_dyn_fix_bottom()
        self.place_dyn_link_bottom()

    def place_dyn_ctr_bottom(self):
        row = len(self.selected_ctr_vars) + 1
        self.dyn_ctr_bottom['opt'] = self.set_optionmenu(self.ctr_vars_frame,
                                                         self.dyn_ctr_bottom['opt'],
                                                         self.dyn_ctr_bottom['var'],
                                                         self.learning_client.get_selectable_vars())
        self.dyn_ctr_bottom['opt'].grid(row=1+row, column=0, sticky="we", padx=2, pady=2)
        self.dyn_ctr_bottom['min'].grid(row=1+row, column=1)
        self.dyn_ctr_bottom['val'].grid(row=1+row, column=2)
        self.dyn_ctr_bottom['max'].grid(row=1+row, column=3)
        self.dyn_ctr_bottom['add'].grid(row=1+row, column=4, sticky="we", padx=2, pady=2)

    def place_dyn_fix_bottom(self):
        row = len(self.selected_fix_vars) + 1
        self.dyn_fix_bottom['opt'] = self.set_optionmenu(self.fix_vars_frame,
                                                         self.dyn_fix_bottom['opt'],
                                                         self.dyn_fix_bottom['var'],
                                                         self.learning_client.get_selectable_vars())
        self.dyn_fix_bottom['opt'].grid(row=1+row, column=0, sticky="we", padx=2, pady=2)
        self.dyn_fix_bottom['val'].grid(row=1+row, column=1)
        self.dyn_fix_bottom['add'].grid(row=1+row, column=2, sticky="we", padx=2, pady=2)

    def place_dyn_link_bottom(self):
        row = len(self.selected_link_vars) + 1
        self.dyn_link_bottom['opt1']= self.set_optionmenu(self.link_vars_frame,
                                                          self.dyn_link_bottom['opt1'],
                                                          self.dyn_link_bottom['var1'],
                                                          self.learning_client.get_selectable_vars())
        self.dyn_link_bottom['opt2']= self.set_optionmenu(self.link_vars_frame,
                                                          self.dyn_link_bottom['opt2'],
                                                          self.dyn_link_bottom['var2'],
                                                          self.learning_client.get_linkable_vars())
        self.dyn_link_bottom['opt1'].grid(row=1+row, column=0, sticky="we", padx=2, pady=2)
        self.dyn_link_bottom['opt2'].grid(row=1+row, column=1, sticky="we", padx=2, pady=2)
        self.dyn_link_bottom['add'].grid(row=1+row, column=2, sticky="we", padx=2, pady=2)

    def set_optionmenu(self, frame, optionmenu, var, options):
        if options:
            options.sort()
            var.set('Select Variable')
            optionmenu.destroy()
            optionmenu = tk.OptionMenu(frame, var, *options)
            optionmenu.config(width=11, font=small_font)
        else:
            var.set('Not Available')
            optionmenu.config(width=11, font=small_font, state='disabled')
        return optionmenu

    def show_current_dyn_reconfig(self):
        self.learning_client.refresh_current_config()
        for var, items in (self.selected_ctr_vars + self.selected_fix_vars):
            items['val'].delete(0, 'end')
            items['val'].insert(0, str(self.learning_client.current_config[var]))

    def restore_dyn_reconfig_default(self):
        self.learning_client.default_dyn_reconfig()
        self.parent.after(10, self.show_current_dyn_reconfig)

    def set_dyn_reconfig_max(self):
        if self.learning_initialized:
            point = self.bo.space.max_point()['max_params']
            self.learning_client.send_controlled_vars(**point)
            self.parent.after(10, self.show_current_dyn_reconfig)

    def send_dyn_reconfig_vals(self):
        for var, items in (self.selected_ctr_vars + self.selected_fix_vars):
            self.learning_client.set_dyn_reconfig_var(var, self.get_float_entry(items['val']))
        self.parent.after(10, self.show_current_dyn_reconfig)
