import igl
from scipy.optimize import minimize
import numpy as np
import sys
import polyscope as ps
from matplotlib.pyplot import get_cmap
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.use('tkagg')

visualize = False

def rigid_alignment(dir_0='./data/download/rose_0.obj', dir_1='./data/download/rose_1.obj'):
    v_0, _, _, f_0, _, _ = igl.read_obj(dir_0, float)
    v, _, _, f, _, _ = igl.read_obj(dir_1, float)

    def dist(A):
        ax = A[0]
        ay = A[1]
        az = A[2]
        T = A[3:]
        r = Rotation.from_euler('xyz', [ax, ay, az])
        R = r.as_matrix()
        return ((v_0 @ R + T - v) ** 2).sum()

    sol = minimize(dist, np.array([0, 0, 0, 0, 0, 0]))

    x = sol.x
    ax = x[0]
    ay = x[1]
    az = x[2]
    T = x[3:]
    r = Rotation.from_euler('xyz', [ax, ay, az])
    R = r.as_matrix()

    dist = np.sqrt(((v_0 @ R + T - v) ** 2).sum(1))

    dist_mean = np.sqrt(((v_0 @ R + T - v) ** 2).sum(1).mean())

    return dist_mean, dist, R, T, v_0, v, f_0, f

def calc(dir_0, dir_1):
    dist_mean, dist, R, T, v_0, v, f_0, f = rigid_alignment('./data/download/{}'.format(dir_0), './data/download/{}'.format(dir_1))

    try:
        if visualize:
            ps.init()
    except:
        pass

    cmap = get_cmap('RdBu')
    c = cmap((1 - dist/dist.max()) * 0.99)[:, :3]

    if visualize:
        m00 = ps.register_surface_mesh('m_00_', v_0, f_0)
        m0 = ps.register_surface_mesh('m_0_', v_0 @ R + T, f_0)
        m = ps.register_surface_mesh('m', v, f)
        m0.add_color_quantity('color', c, enabled=True)

    vs = np.concatenate([v_0 @ R + T, v])
    es = np.arange(len(v_0)).reshape(-1, 1)
    es = np.concatenate([es, es + len(v_0)], 1)

    es = es[np.arange(0, len(es), 3)]

    vs_new = []
    es_new = []

    for e in es:
        v0 = vs[e[0]]
        v1 = vs[e[1]]
        if tuple(v0) not in vs_new:
            vs_new.append(tuple(v0))
        iv_0 = vs_new.index(tuple(v0))

        if tuple(v1) not in vs_new:
            vs_new.append(tuple(v1))
        iv_1 = vs_new.index(tuple(v1))

        es_new.append([iv_0, iv_1])

    vs_new = np.array(vs_new)
    es_new = np.array(es_new)

    obj_lines = []

    for i, v in enumerate(v_0):
        obj_lines.append("v {} {} {} {} {} {}\n".format(v[0], v[1], v[2], c[i][0], c[i][1], c[i][2]))

    for face in f:
        obj_lines.append("f {} {} {}\n".format(face[0]+1, face[1]+1, face[2]+1))

    obj_str = ''.join(obj_lines)

    with open('rose_compare.obj', 'w') as ofile:
        ofile.write(obj_str)


    if visualize:
        ps.register_curve_network('dist', vs_new, es_new)

        # ps.register_point_cloud('v_0_', v_0 @ R + T)
        # ps.register_point_cloud('v', v)

        ps.show()
    return dist_mean

ks_e = []
k_e = 0.00625
for i in range(10):
    ks_e.append(k_e)
    k_e *= 2

ks_b = []
k_b = 0.001
for i in range(10):
    ks_b.append(k_b)
    k_b *= 10

if False:
    mat = []
    name = "rose"
    for k_e in ks_e:
        for k_b in ks_b:
            dir_1 = "{}_{:6f}_{:6f}.obj".format(name, k_e, k_b)
            dir_1 = "xmas_truth.obj".format(name, k_e, k_b)
            dir_0 = "{}_truth.obj".format(name)
            dir_0 = "xmas_out.obj".format(name)
            dist_mean = calc(dir_0=dir_0, dir_1=dir_1)
            print(dir_1, dist_mean)
            mat.append(dist_mean)


from mpl_toolkits.axes_grid1 import make_axes_locatable


# mat = np.array(mat).reshape(len(ks_e), len(ks_b))
# np.save('out', mat)
mat = np.load('out.npy')
print(mat)

plt.figure(figsize=(8, 8))
ax = plt.subplot()
im = ax.imshow(mat, cmap='RdBu_r', interpolation='nearest', extent=[-3, 6, -1, 1], aspect='auto', vmin=0, vmax=23.0)


@plt.FuncFormatter
def fake_log(x, pos):
    'The two args are the value and tick position'
    return r'$10^{%d}$' % (x)
ax.xaxis.set_major_formatter(fake_log)
ax.yaxis.set_major_formatter(fake_log)
ax.set_ylabel('weight of electrostatic energy')
ax.set_xlabel('weight of bending energy')

# ax.set_xscale('log')
# ax.set_yscale('log')
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="5%", pad=0.05)

plt.colorbar(im, cax=cax, label="shape error(%)")
plt.show()

calc(dir_0="xmas_truth.obj", dir_1="xmas_out.obj")
