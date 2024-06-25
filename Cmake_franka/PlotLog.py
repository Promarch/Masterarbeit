# %%
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# %%

def plot7(df, columns = None):
    if "time" in df.columns:
        x = df["time"]
    else:
        x = df.index.values

    if columns == None:
        columns = np.arange(7)
    
    fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
    
    for i, col in enumerate(columns):
        axs[i].plot(x, df[col].to_numpy())
        axs[i].set_ylabel(col)
        axs[i].grid(True)
        #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])


    axs[-1].set_xlabel('Time')
    # Adjust layout
    plt.tight_layout()

    # Show the plot
    plt.show()

# %%
filePath = "/home/alexandergerard/Masterarbeit/Cmake_franka/"
fileName = "build/tau_data_20240625_155605.txt"
df_orig = pd.read_csv(filePath+fileName, header=None)
df = df_orig.copy()

plot7(df)
# %%
# Import csv file
filePath = "/tmp/libfranka-logs/"
fileName = "log-2024-06-25-11-06-03-763.csv"
df_orig = pd.read_csv(filePath+fileName)

df = df_orig.copy()
df.head(5)
# %%

useCols_cmd_tau_J_d = ["time", "cmd.tau_J_d[0]", "cmd.tau_J_d[1]", "cmd.tau_J_d[2]", "cmd.tau_J_d[3]", "cmd.tau_J_d[4]", "cmd.tau_J_d[5]", "cmd.tau_J_d[6]"]
useCols_state_tau_J = ["time", "state.tau_J[0]", "state.tau_J[1]", "state.tau_J[2]", "state.tau_J[3]", "state.tau_J[4]", "state.tau_J[5]", "state.tau_J[6]"]
useCols_state_tau_ext = ["time", "state.tau_ext_hat_filtered[0]", "state.tau_ext_hat_filtered[1]", "state.tau_ext_hat_filtered[2]", "state.tau_ext_hat_filtered[3]", "state.tau_ext_hat_filtered[4]", "state.tau_ext_hat_filtered[5]", "state.tau_ext_hat_filtered[6]"]
useCols_state_q_d = ["time", "state.q_d[0]", "state.q_d[1]", "state.q_d[2]", "state.q_d[3]", "state.q_d[4]", "state.q_d[5]", "state.q_d[6]"]
useCols = useCols_state_tau_J

fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)

for i, col in enumerate(useCols[1:]):
    axs[i].plot(df["time"].to_numpy(), df[col].to_numpy())
    axs[i].set_ylabel(col)
    axs[i].grid(True)
    #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])


axs[-1].set_xlabel('Time')
# Adjust layout
plt.tight_layout()

# Show the plot
plt.show()
# %%
