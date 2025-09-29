import pandas as pd
import matplotlib.pyplot as plt

def plot_csv_columns(csv_file, keys):
    """
    从 CSV 文件绘制多条曲线，每条曲线单独一个子图
    :param csv_file: CSV 文件路径
    :param keys: 需要绘制的列名列表
    """
    df = pd.read_csv(csv_file)

    fig, axs = plt.subplots(len(keys), 1, figsize=(10, 3 * len(keys)), sharex=True)

    if len(keys) == 1:
        axs = [axs]  # 保证 axs 是可迭代的

    for ax, key in zip(axs, keys):
        data = df[key]
        label = key
        # 如果需要，将节转换为英尺每秒
        if convert_kts_to_fps:
            data = data * 1.68781
            label = key.replace('kts', 'ft/s')  # 把标签里的 kts 改为 ft/s
            ax.set_ylabel(label)
        else:
            ax.set_ylabel(key)

        ax.plot(df.index, data, label=label)  # 用修改后的 label
        ax.legend()
        ax.grid(True)

    axs[-1].set_xlabel('Sample Index')
    plt.tight_layout()
    plt.show()


# 使用示例
if __name__ == "__main__":
    # plot_csv_columns(
    #     'c310_aptest.csv',
    #     [
    #         'ap/airspeed_setpoint',
    #         'velocities/vc-kts',
    #         'ap/airspeed-error-throttle',
    #         'ap/airspeed-throttle-ff',
    #         'ap/airspeed-pi-throttle',
    #         'fcs/throttle-cmd-norm-temp',
    #         'ap/airspeed_hold',
    #         'fcs/throttle-cmd-norm[0]',
    #         'position/h-agl-ft'

    #     ]
    # )
    convert_kts_to_fps = True
    plot_csv_columns(
        './jsbsim/c310_land.csv',
        [
            'u_fps',
            'altitude_ft',
            'position/long-gc-rad',
            'position/lat-geod-rad'

        ]
    )
