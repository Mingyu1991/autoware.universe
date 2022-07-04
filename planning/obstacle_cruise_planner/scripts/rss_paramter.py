import matplotlib.pyplot as plt
import numpy as np

def calc_rss_distance(t_idling, v_ego, a_ego, v_obj, a_obj, margin=6.0):
    return v_ego * t_idling + v_ego**2 / 2.0 / abs(a_ego) - v_obj**2 / 2.0 / abs(a_obj) + margin

def policy1(v_ego, v_obj):
    t_idling = 3.5
    a_ego = -1.0
    a_obj = -1.0
    return calc_rss_distance(t_idling, v_ego, a_ego, v_obj, a_obj)

def policy2(v_ego, v_obj):
    t_idling = 1.5
    a_ego = -0.5
    a_obj = -1.0
    return calc_rss_distance(t_idling, v_ego, a_ego, v_obj, a_obj)

def policy3(v_ego, v_obj):
    t_idling = 2.5
    a_ego = -0.6
    a_obj = -0.8
    return calc_rss_distance(t_idling, v_ego, a_ego, v_obj, a_obj)

print("Policy details")
print("[policy1] t_idling: {}, a_ego: {}, a_obj: {}".format(3.5, -1.0, -1.0))
print("[policy2] t_idling: {}, a_ego: {}, a_obj: {}".format(1.5, -0.5, -1.0))
print("[policy3] t_idling: {}, a_ego: {}, a_obj: {}".format(2.5, -0.6, -0.8))

fig = plt.figure()

# Object and ego are the same velocity
vs_ego = [val for val in np.arange(0, 12, 0.01)]
ax = fig.add_subplot(211)
ax.set_title("Object and ego are the same velocity")
ax.set_xlabel("ego vel [m/s]")
ax.set_ylabel("rss distance [m]")
ax.plot(vs_ego, [policy1(v_ego, v_ego) for v_ego in vs_ego], label="policy1")
ax.plot(vs_ego, [policy2(v_ego, v_ego) for v_ego in vs_ego], label="policy2")
ax.plot(vs_ego, [policy3(v_ego, v_ego) for v_ego in vs_ego], label="policy3")
ax.legend()

# Object's velocity is half of ego's one
ax.set_title("Object's velocity is half of ego's one")
vs_ego = [val for val in np.arange(10 / 3.6 * 2, 12, 0.01)]
ax = fig.add_subplot(212)
ax.set_xlabel("ego vel [m/s]")
ax.set_ylabel("rss distance [m]")
ax.plot(vs_ego, [policy1(v_ego, v_ego / 2.0) for v_ego in vs_ego], label="policy1")
ax.plot(vs_ego, [policy2(v_ego, v_ego / 2.0) for v_ego in vs_ego], label="policy2")
ax.plot(vs_ego, [policy3(v_ego, v_ego / 2.0) for v_ego in vs_ego], label="policy3")
ax.legend()

plt.show()
