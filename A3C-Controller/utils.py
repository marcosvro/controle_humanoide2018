from torch import nn
import torch
import numpy as np
from parameters import *

def v_wrap(np_array, dtype=np.float32):
    if np_array.dtype != dtype:
        np_array = np_array.astype(dtype)
    return torch.from_numpy(np_array)


def set_init(layers):
    for layer in layers:
        nn.init.normal_(layer.weight, mean=0., std=0.1)
        nn.init.constant_(layer.bias, 0.)


def push_and_pull(opt, lnet, gnet, done, s_, bs, ba, br, gamma):
    if done:
        v_s_ = 0.               # terminal
    else:
        v_s_ = lnet.forward(v_wrap(s_[None, :]))[-1].data.numpy()[0, 0]

    buffer_v_target = []
    for r in br[::-1]:    # reverse buffer r
        v_s_ = r + gamma * v_s_
        buffer_v_target.append(v_s_)
    buffer_v_target.reverse()

    loss, c_loss, a_loss, adv = lnet.loss_func(
        v_wrap(np.vstack(bs)),
        v_wrap(np.array(ba), dtype=np.int64) if ba[0].dtype == np.int64 else v_wrap(np.vstack(ba)),
        v_wrap(np.array(buffer_v_target)[:, None]))

    # calculate local gradients and push local parameters to global
    opt.zero_grad()
    loss.backward()
    for lp, gp in zip(lnet.parameters(), gnet.parameters()):
        gp._grad = lp.grad
    opt.step()

    # pull global parameters
    lnet.load_state_dict(gnet.state_dict())

    return loss.detach() ,c_loss, a_loss, adv


def record(global_ep, global_ep_r, stats, best_ep_r, name, state_dicts):
    with global_ep.get_lock():
        global_ep.value += 1
        ep = global_ep.value
        with global_ep_r.get_lock():
            if global_ep_r.value == 0.:
                global_ep_r.value = stats['reward']
            else:
                global_ep_r.value = global_ep_r.value * 0.99 + stats['reward'] * 0.01
            with best_ep_r.get_lock():
                saved = False
                if global_ep_r.value > best_ep_r.value:
                    best_ep_r.value = global_ep_r.value
                    torch.save(state_dicts, LOG_DIR)
                    saved = True

    saved_msg = "New ep reward record" if saved else ""
    vetor_reward = np.around([stats['ProgressReward'],
                    stats['PoseReward'],
                    stats['IncReward'],
                    stats['OriReward']], decimals=1)
    print(
        name,
        "Ep:", ep,
        "| Ep_r: %.1f" % stats['reward'],
	"| Ep_r: ", vetor_reward,
        "| time: %.1f" % stats['TimeEpisodeDuration'],
        "| Global Ep_r: %.1f" % global_ep_r.value,
        "| Best Global Ep_r: %.1f" % best_ep_r.value,
        saved_msg,
    )
    
    return ep
