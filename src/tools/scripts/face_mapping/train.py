import pandas as pd
import neurolab as nl
from ffsignal import blend_shape_names
from fit import SHKEY_DEPS, get_motors
import pickle

def read_data(data_dir, shape_keys):
    if isinstance(shape_keys, basestring):
        shape_keys = [shape_keys]
    motors = get_motors('head.yaml')
    motor_dict = {motor['name']: motor for motor in motors}
    ff_df, signal_df = pd.DataFrame(), pd.DataFrame()
    for shape_key in shape_keys:
        ff_file = '{}/ff_data_{}.csv'.format(data_dir, shape_key)
        signal_file = '{}/signal_data_{}.csv'.format(data_dir, shape_key)
        tmp_ff_df = pd.read_csv(ff_file, names=blend_shape_names)
        ff_df = pd.concat([ff_df, tmp_ff_df], ignore_index=True)
        tmp_signal_df = pd.read_csv(signal_file, names=SHKEY_DEPS[shape_key])
        remainder_names = motor_dict.keys()[:]
        for name in SHKEY_DEPS[shape_key]:
            remainder_names.remove(name)
        init_values = [motor_dict[name]['init'] for name in remainder_names]
        complementary_signal_df = pd.DataFrame(
            [init_values]*len(tmp_signal_df), columns=remainder_names)
        tmp_signal_df = pd.concat(
            [tmp_signal_df, complementary_signal_df], axis=1)
        signal_df = pd.concat([signal_df, tmp_signal_df], ignore_index=True)

    return ff_df, signal_df

if __name__ == '__main__':
    data_dir = 'training_data'
    #shape_keys = ['EyeBlink_L', 'EyeBlink_R']
    shape_keys = ['EyeBlink_L']
    model_file = 'net.model'
    norm_file = 'norm.model'
    active_motor_file = 'motors.model'
    ff_df, signal_df = read_data(data_dir, shape_keys)

    # Normalize
    signal_norm_df = signal_df.sub(signal_df.min()).div(signal_df.max()-signal_df.min())
    signal_norm_df = signal_norm_df.dropna(axis=1)
    with open(active_motor_file, 'w') as f:
        f.write('\n'.join(signal_norm_df.columns.tolist()))

    # Train and save the model
    net = nl.net.newff([[0, 1]]*len(blend_shape_names), [10, signal_norm_df.shape[1]])
    err = net.train(ff_df.as_matrix(), signal_norm_df.as_matrix(), show=1)
    with open(model_file, 'w') as f:
        pickle.dump(net, f)
    norm_df = pd.DataFrame({'min': signal_df.min(), 'max': signal_df.max()})
    norm_df.to_csv(norm_file)

