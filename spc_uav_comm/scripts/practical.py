"""
This module contains the functions necessary to extract and graph data saved by the learning GUI
Author : Asem Khattab
Eamil : asem.khattab@gmail.com
"""
import pickle
import numpy as np
import matplotlib.pyplot as plt


def fname(shape, dim, fric, init=1, speed=0.2):
    '''
    :param shape: trajectory shape
    :param dim: dimension of the learning problem
    :param fric: friction coefficient
    :param init: initialization (files with same 'init' are initialized with the same points
    :param speed: speed of the drawing
    :return: the name of the file containing simulation results
    '''
    return shape + str(dim) + 'd' + str(fric) + '-' + str(init) + '-2s' + str(speed)


def load_results(fname):
    '''
    :param fname: name of the file containing simulation results
    :return: simulation results data (exception if file doesn't exist)
    '''
    f = open('practical//' + fname + '.pkl', 'rb')
    data = pickle.load(f)
    f.close()
    return data


def extract_data(data):
    '''
    :param data: simulation data (as returned by 'load_results'
    :return: a dictionary of data series as the number of iteration increases starting from the number of initial observations
    '''
    reward = -100*np.sum(data['pos_error'], 1) - 150 * np.sum(data['rot_error'], 1) - 0.15* np.sum(data['x'], 1) /data['x'].shape[1]
    # reward = -np.sum(data['pos_error'], 1) - 1.5 * np.sum(data['rot_error'], 1) - np.sum(data['x'], 1) / (data['x'].shape[1]*10)
    arg_max = []
    maximum = []
    max_reward = []
    avrg = []
    avrg_reward = []
    dist = []
    for i in range(data['init_pts_num'], len(data['y']) + 1):
        arg_max.append(np.argmax(data['y'][0:i]))
        max_reward.append(reward[arg_max[-1]])
        maximum.append(data['y'][arg_max[-1]])
        avrg.append(np.mean(data['y'][0:i]))
        avrg_reward.append(np.mean(reward[0:i]))
        dist.append(np.sqrt(min(np.sum((data['x'][0:i-1] - data['x'][i-1])**2, 1))))
    extracted = {'arg_max': arg_max,
                 'maximum': maximum,
                 'max_reward': max_reward,
                 'avrg': avrg,
                 'avrg_reward': avrg_reward,
                 'reward': reward}
    return extracted


def get_info(fname):
    '''
    :param fname: name of the file containing simulation results
    :return: a dictionary containing a summary of training results for default values
    (the first initial observation), after initialization and after training.
    '''
    data = load_results(fname)
    extracted = extract_data(data)

    def info(i):
        return {'x': zip(data['keys'], data['x'][i]),
                'xbar': sum(data['x'][i])/len(data['x'][i]),
                'i': i,
                'target': data['y'][i],
                'reward': extracted['reward'][i],
                'pos_error': sum(data['pos_error'][i]),
                'rot_error': data['rot_error'][i]}
    return {'default': info(0), 'init': info(extracted['arg_max'][0]), 'after': info(extracted['arg_max'][40])}


def load_traj(shape, dim, fric, i):
    '''
    :param shape: trajectory shape
    :param dim: dimension of the learning problem
    :param fric: friction coefficient
    :param i: iteration
    :return: lists of the set positions and real positions throughout the training iteration.
    '''
    f = open('practical//traj-' + shape + str(dim) + 'd' + str(fric) + '-' + str(i) + '.pkl', 'rb')
    set_pos, real_pos = pickle.load(f)
    f.close()
    return set_pos, real_pos


def vsubplots(n, ylims, ylabels, **kw):
    '''
    :param n: number of plots
    :param ylims: list of tuples representing limits of each plot in the y-axis
    :param ylabels: list of labels of the plot
    :param kw: key words options of the plt.subplots function (look at the documentation of that function)
    :return: handels of the subplots
    '''
    f, axes = plt.subplots(n, 1, **kw)
    for i in range(n):
        if ylims[i] is not None: axes[i].set_ylim(ylims[i])
        axes[i].set_ylabel(ylabels[i])
    return f, axes


def plot_coefs(coefs, shape, dim, init=1, speed=0.2, feature='maximum', style='', legend=True, ax=None):
    '''
    graphs a certain feature for multiple training that only differ in the friction coefficient
    :param coefs: list of friction coefficient
    :param shape: drawing shape ('circle' or 'square')
    :param dim: dimension of the learning
    :param init: initialization (files with same 'init' are initialized with the same points
    :param speed: speed of drawing
    :param feature: the feature to be graphed (can be any of the ones in the dictionary returned by 'extract_data')
    :param style: drawing style string (see plt.plot()) documentation) if style is a string,
    the same style will be used for all lines, if it is a list of strings,
    they will be applied in order to the different lines
    :param legend: boolean to enable including a legend for the friction coefficient.
    :param ax: axis handel to draw the graph in. If its not provided, the graph will be drawn in a new window.
    :return: axis handel of the drawn graph (can be used to draw other lines in the same graph).
    '''
    if not ax:
        f, ax = plt.subplots(figsize=(4, 3))
    else:
        ax.set_prop_cycle(None)
    for i in range(len(coefs)):
        data = load_results(fname(shape, dim, coefs[i], init, speed))
        numIter = len(extract_data(data)[feature])
        numInitObser = data['init_pts_num']
        label = r'$\mu$' + '=' + str(coefs[i]) if legend else None
        s = style if type(style) is str else style[i] if len(style) == len(coefs) else ''
        ax.plot(list(range(numInitObser, numInitObser + numIter)), extract_data(data)[feature][0:numIter], s, label=label)
    ax.set_xlim([data['init_pts_num'], numInitObser + numIter-1])
    ax.grid(1)
    return ax


#------------------------------- The graphs in the thesis -------------------------------#

coefs = [0.4, 0.8, 1.16, 1.4]
styles = ['', '--', '-.', ':']
ylabels = [r'$S_n^+$', r'$y_n^+$', r'$\bar{S}_n$']
ylims = [(0.5, 0.97), (-13, -7.5), (0.1, 0.65)]

f, axs = plt.subplots(1, 2, figsize=(8,3), sharey=True)
plot_coefs(coefs, 'circle', 4, ax=axs[0], style=styles, legend=True, feature='max_reward')
axs[0].set_xlabel('function evaluations')
axs[0].set_ylabel('maximum reward')
axs[0].set_title('circle')
plot_coefs(coefs, 'square', 4, ax=axs[1], style=styles, legend=False, feature='max_reward')
axs[1].set_xlabel('function evaluations')
axs[1].set_title('square')
f.tight_layout(rect=(0, 0, 1, 0.94))
f.legend(loc='upper center', ncol=5, frameon=False)


f, axs = vsubplots(3, ylims, ylabels, sharex=True, figsize=(5, 6))
plot_coefs(coefs, 'circle', 2, ax=axs[0], style=styles)
plot_coefs(coefs, 'circle', 2, ax=axs[1], style=styles, legend=False, feature='max_reward')
plot_coefs(coefs, 'circle', 2, ax=axs[2], style=styles, legend=False, feature='avrg')
axs[-1].set_xlabel(r'$n$')
f.tight_layout(rect=(0, 0, 1, 0.97))
f.legend(loc='upper center', ncol=5, frameon=False)


f, axs = vsubplots(2, ylims, ylabels, sharex=True, figsize=(5, 4.2))
plot_coefs(coefs, 'circle', 4, ax=axs[0], style=styles)
plot_coefs(coefs, 'circle', 4, ax=axs[1], style=styles, legend=False, feature='max_reward')
axs[-1].set_xlabel(r'$n$')
f.tight_layout(rect=(0, 0, 1, 0.97))
f.legend(loc='upper center', ncol=5, frameon=False)


f, axs = vsubplots(2, 2*[None], ylabels, sharex=True, figsize=(5, 4.2))
plot_coefs(coefs, 'circle', 4, ax=axs[0], style=styles[0])
plot_coefs(coefs, 'circle', 4, ax=axs[1], style=styles[0], legend=False, feature='max_reward')
plot_coefs(coefs, 'circle', 4, speed=0.5, ax=axs[0], style=styles[1], legend=False)
plot_coefs(coefs, 'circle', 4, speed=0.5, ax=axs[1], style=styles[1], legend=False, feature='max_reward')
axs[-1].set_xlabel(r'$n$')
f.tight_layout(rect=(0, 0, 1, 0.97))
f.legend(loc='upper center', ncol=5, frameon=False)


f, axs = vsubplots(2, ylims, ylabels, sharex=True, figsize=(5, 4.2))
plot_coefs(coefs, 'square', 4, ax=axs[0], style=styles)
plot_coefs(coefs, 'square', 4, ax=axs[1], style=styles, legend=False, feature='max_reward')
axs[-1].set_xlabel(r'$n$')
f.tight_layout(rect=(0, 0, 1, 0.97))
f.legend(loc='upper center', ncol=5, frameon=False)


f, axs = vsubplots(4, 4*[None], 4*[r'$y_n^+$'], sharex=True, figsize=(6, 7))
f, axs = plt.subplots(1, 2, figsize=(8,3))
coefs=[0.4, 1.16]
styles = ['k', 'k--', 'r-.', 'r:']
for i in range(len(coefs)):
    axs[i].set_title(r'$\mu=$'+str(coefs[i]))
    plot_coefs([coefs[i]], 'circle', 4, ax=axs[i], style=styles[0], feature='max_reward', legend=False)
    plot_coefs([coefs[i]], 'circle', 4, 11, ax=axs[i], style=styles[1], feature='max_reward', legend=False)
    plot_coefs([coefs[i]], 'square', 4, ax=axs[i], style=styles[2], feature='max_reward', legend=False)
    plot_coefs([coefs[i]], 'square', 4, 11, ax=axs[i], style=styles[3], feature='max_reward', legend=False)
labels = [r'Circle With Logistic Function', r'Circle', r'Square With Logistic Function', r'Square']
for i in range(len(axs[0].lines)):
    axs[0].lines[i].set_label(labels[i])
axs[0].set_xlabel(r'number of evaluations')
axs[1].set_xlabel(r'number of evaluations')
f.tight_layout(rect=(0, 0, 1, 0.88))
f.legend(loc='upper center', ncol=2, frameon=False)


f, axs = vsubplots(4, 4 * [None], 4*[''], sharex=True, figsize=(4, 7))
f2, axs2 = vsubplots(4, 4 * [None], 4*[''], sharex=True, figsize=(4, 7))
styles = ['r', 'k--']
for i in range(len(coefs)):
    axs[i].set_title(r'$\mu=$'+str(coefs[i]))
    axs2[i].set_title(r'$\mu=$'+str(coefs[i]))
    plot_coefs([coefs[i]], 'circle', 4, ax=axs[i], style=styles[0], feature='max_reward', legend=False)
    plot_coefs([coefs[i]], 'circle', 4, 111, ax=axs[i], style=styles[1], feature='max_reward', legend=False)
    plot_coefs([coefs[i]], 'circle', 4, ax=axs2[i], style=styles[0], feature='avrg_reward', legend=False)
    plot_coefs([coefs[i]], 'circle', 4, 111, ax=axs2[i], style=styles[1], feature='avrg_reward', legend=False)
f.suptitle(r'$y_n^+$'+ ' curves')
f2.suptitle(r'$\bar{y}_n$'+ ' curves')
for axss in [axs, axs2]:
    axss[-1].set_xlabel(r'$n$')
for fs in [f, f2]:
    fs.tight_layout(rect=(0, 0, 1, 0.96))


# initial points for the 2D case
data = load_results(fname('circle', 2, 0.8))
plt.figure()
plt.plot(data['x'][0:10,1], data['x'][0:10,0], 'xr')
plt.axis('square')
plt.axis([0, 50, 0, 50])
plt.xlabel(r'$k_t^{1,1}$')
plt.ylabel(r'$k_t^{2,2} = k_t^{3,3}$')


# logistic function for different k
x = np.arange(-10,10,0.01)
plt.figure()
plt.plot(x, 1/(1+np.exp(-0.5*x)), label=r'$l=0.5$')
plt.plot(x, 1/(1+np.exp(-x)), '--', label=r'$l=1$')
plt.plot(x, 1/(1+np.exp(-2*x)), ':', label=r'$l=2$')
plt.legend()
plt.grid()
plt.ylim(0, 1)
plt.xlim(-10,10)
plt.xlabel(r'$x$')

##################### Preparing Tables #####################

# table for learning results
for stage in ['default', 'init', 'after']:
    v2 = []
    v4 = []
    for coef in [0.4, 0.8, 1.16, 1.4]:
        info = get_info(fname('circle',2, coef))
        v2+= [info[stage]['reward'], 100*info[stage]['pos_error'], info[stage]['xbar']]
        info = get_info(fname('circle', 4, coef))
        v4 += [info[stage]['reward'], 100*info[stage]['pos_error'], info[stage]['xbar']]
    print ' & '.join(['%.2f' % round(elem, 2) for elem in v2]) + '\\\\'
    print ' & '.join(['%.2f' % round(elem, 2) for elem in v4]) + '\\\\'


# table for learned values
for coef in [0.4, 0.8, 1.16, 1.4]:
    info = dict(get_info(fname('square',4, coef))['after']['x'])
    v4 = [coef, info['c_KP_t_xx'], info['c_KP_t_yy'], info['K_v_x'], info['K_v_y']]
    print ' & '.join(['%.3f' % round(elem, 3) for elem in v4]) + '\\\\'


##################### Drawing Trajectories #####################

# Learning results for circle
fig = plt.figure(figsize=(5,5))
set_pos, real_pos = load_traj('circle',4,1.4,0)
plt.plot(real_pos[:,1],real_pos[:,2], '-.', linewidth=2.0, label='Default Values, '+r'$E_t=0.1215$')
set_pos, real_pos = load_traj('circle',4,1.4,10)
plt.plot(real_pos[:,1],real_pos[:,2], '--', linewidth=2.0, label='After Initialization, '+r'$E_t=0.0667$')
set_pos, real_pos = load_traj('circle',4,1.4,50)
plt.plot(real_pos[:,1],real_pos[:,2] , linewidth=2.0, label='After Training, '+r'$E_t=0.0172$')
plt.plot(set_pos[:,1],set_pos[:,2], 'k:', label='Command Trajectory')
plt.axis('square')
plt.axis([-1.1, 1.1, 0.9, 3.1])
plt.gca().invert_xaxis()
plt.xlabel(r'$y$')
plt.ylabel(r'$z$')
plt.legend()
fig.tight_layout()


# Learning results for square
fig = plt.figure(figsize=(5,5))
set_pos, real_pos = load_traj('square',4,1.4,0)
plt.plot(real_pos[:,1],real_pos[:,2], '-.', linewidth=2.0, label='Default Values, '+r'$E_t=0.0937$')
set_pos, real_pos = load_traj('square',4,1.4,10)
plt.plot(real_pos[:,1],real_pos[:,2], '--', linewidth=2.0, label='After Initialization, '+r'$E_t=0.0480$')
set_pos, real_pos = load_traj('square',4,1.4,50)
plt.plot(real_pos[:,1],real_pos[:,2], linewidth=2.0, label='After Training, '+r'$E_t=0.0215$')
plt.plot(set_pos[:,1],set_pos[:,2], 'k:', label='Command Trajectory')
plt.axis('square')
plt.axis([-1.1, 1.1, 0.9, 3.1])
plt.gca().invert_xaxis()
plt.xlabel(r'$y$')
plt.ylabel(r'$z$')
plt.legend()
fig.tight_layout()



#------------------------------- Ramy Preliminary Test Graphs -------------------------------#

# Learning results for circle
fig = plt.figure(figsize=(5,5))
set_pos, real_pos = load_traj('circle',2,1.16,0)
plt.plot(real_pos[:,1],real_pos[:,2], '-.', linewidth=2.0, label='Default Values,')
set_pos, real_pos = load_traj('circle',2,1.16,50)
plt.plot(real_pos[:,1],real_pos[:,2], linewidth=2.0, label='After Training')
plt.plot(set_pos[:,1],set_pos[:,2], 'k:', label='Command Trajectory')
plt.axis('square')
plt.axis([-0.5, 0.5, 1.5, 2.5])
plt.gca().invert_xaxis()
plt.xlabel(r'$y$')
plt.ylabel(r'$z$')
plt.legend()
fig.tight_layout()




speed_ = 0.2
coefs = [1.16]
styles = ['']
ylabels = [r'$S_n^+$', r'$y_n^+$', r'$\bar{S}_n$']
ylims = [(0.4, 0.7), (-40, -20), (0, 0.3)]

f, axs = vsubplots(3, ylims, ylabels, sharex=True, figsize=(5, 6))
plot_coefs(coefs, 'circle', 2,speed = speed_, ax=axs[0], style=styles)
plot_coefs(coefs, 'circle', 2,speed = speed_, ax=axs[1], style=styles, legend=False, feature='max_reward')
plot_coefs(coefs, 'circle', 2,speed = speed_, ax=axs[2], style=styles, legend=False, feature='avrg')
axs[-1].set_xlabel(r'$n$')
f.tight_layout(rect=(0, 0, 1, 0.97))




# table for learned values
for coef in [1.16]:
    info = dict(get_info(fname('circle',2, coef))['after']['x'])
    v4 = [coef, info['c_KP_t_yy'], info['K_v_y']]
    print ' & '.join(['%.3f' % round(elem, 3) for elem in v4]) + '\\\\'