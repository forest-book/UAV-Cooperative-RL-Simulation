import csv
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
from typing import List, Dict
import datetime

# データ保存時の日時
current_time = datetime.datetime.now()

class DataLogger:
    """
    シミュレーション中のデータを収集し、CSVファイルに保存するクラス。
    """
    def __init__(self):
        self.data: List = []
        self.timestamp: List[float] = []
        self.uav_trajectories: Dict[str, List[np.ndarray]] = defaultdict(list)

    def logging_timestamp(self, time: float):
        self.timestamp.append(time)

    def logging_uav_trajectories(self, uav_id: int, uav_position: np.ndarray):
        self.uav_trajectories[f"uav{uav_id}_true_pos"].append(uav_position.copy())

    def save_trajectories_data_to_csv(self, filename: str = f'uav_trajectories_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        複数のUAVの軌道(2D)をcsv保存する関数

        Args:
            filename (str): 保存するCSVファイル名
        """
        dir_path = "../data/csv//trajectories/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write headers
            headers = ['time'] + [f'uav{i}_true_pos_x' for i in range(1, 7)] + [f'uav{i}_true_pos_y' for i in range(1, 7)]
            writer.writerow(headers)

            # Write data
            for t, positions in zip(self.timestamp, zip(*[self.uav_trajectories[f'uav{i}_true_pos'] for i in range(1, 7)])):
                row = [t] + [pos[0] for pos in positions] + [pos[1] for pos in positions]
                writer.writerow(row)
        print(f"Data successfully saved to {filename}")



class Plotter:
    @staticmethod
    def plot_trajectories():
        """
        複数のUAVの軌跡を2Dプロットする関数

        Args:
            uav_positions (List[np.ndarray]): 各UAVの位置データのリスト。各要素は(N, 2)の形状を持つnumpy配列。
            labels (List[str]): 各UAVのラベルのリスト。
            title (str): プロットのタイトル。
        """
        # --- 図4(a), (d)相当: 全UAVの軌跡 ---
        plt.figure(figsize=(10, 8))
        for i in range(1, 7):
            positions = np.array(history[f'uav{i}_true_pos'])
            plt.plot(positions[:, 0], positions[:, 1], label=f'UAV {i}')
            plt.scatter(positions[0, 0], positions[0, 1], marker='o', label=f'UAV {i} Start')
            plt.scatter(positions[-1, 0], positions[-1, 1], marker='x', label=f'UAV {i} End')
        plt.title('UAV Trajectories (Scenario: ' + self.params.get('event', 'Continuous') + ')')
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    @staticmethod
    def plot_errors_from_csv(filename: str):
        """
        Plot fusion estimation errors from a CSV file.

        Parameters:
            filename (str): Name of the CSV file to read.
        """

        # Read CSV file
        data = pd.read_csv(filename)

        fig, ax = plt.subplots(figsize=(12, 6))
        colors = {2: 'c', 3: 'b', 4: 'g', 5: 'r', 6: 'm'}

        for i in range(2, 7):
            errors = data[f'uav{i}_fused_error']
            valid_times = data['time'][~errors.isna()]
            valid_errors = errors[~errors.isna()]

            if not valid_errors.empty:
                ax.plot(valid_times, valid_errors, 
                        label=f'$||\pi_{{{i}1}} - \chi_{{{i}1}}||$', 
                        color=colors.get(i, 'k'))

        ax.set_title('Consensus-based RL Fusion Estimation', fontsize=16, fontweight='bold')
        ax.set_xlabel('$k$ (sec)', fontsize=14)
        ax.set_ylabel('$||\pi_{ij}(k) - \chi_{ij}(k)||$ (m)', fontsize=14)
        ax.set_ylim(0, 100.0)
        ax.legend()
        ax.grid(True)

        plt.show()

