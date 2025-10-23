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
        self.timestamp: List[float] = []
        self.uav_trajectories: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.fused_RL_errors: Dict[str, List[np.ndarray]] = defaultdict(list)

    def logging_timestamp(self, time: float):
        self.timestamp.append(time)

    def logging_uav_trajectories(self, uav_id: int, uav_position: np.ndarray):
        self.uav_trajectories[f"uav{uav_id}_true_pos"].append(uav_position.copy())

    def logging_fused_RL_error(self, uav_id: int, error: float):
        self.fused_RL_errors[f"uav{uav_id}_fused_error"].append(error)

    def save_trajectories_data_to_csv(self, filename: str = f'uav_trajectories_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        複数のUAVの軌道(2D)をcsv保存する関数

        Args:
            filename (str): 保存するCSVファイル名
        """
        dir_path = "../data/csv/trajectories/" + filename
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

    def save_fused_RL_errors_to_csv(self, filename: str = f'fused_RL_error_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        相対自己位置の融合推定誤差をcsv保存する関数

        Args:
            filename (str): 保存するCSVファイル名
        """
        dir_path = "../data/csv/RL_errors/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write headers
            headers = ['time'] + [f'uav{i}_fused_error' for i in range(2, 7)]
            writer.writerow(headers)

            # Write data
            for t, errors in zip(self.timestamp, zip(*[self.fused_RL_errors[f'uav{i}_fused_error'] for i in range(2, 7)])):
                row = [t] + list(errors)
                writer.writerow(row)
        print(f"Data successfully saved to {filename}")

class Plotter:
    @staticmethod
    def plot_trajectories_from_csv(filename: str = f'uav_trajectories_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        複数のUAVの軌跡を2Dプロットする関数

        Args:
            filename (str): 読み込むCSVファイル名
        """
        try:
            # Read CSV file
            file_path = f"../data/csv/trajectories/{filename}"
            data = pd.read_csv(file_path)

            plt.figure(figsize=(10, 8))
            for i in range(1, 7):
                x_positions = data[f'uav{i}_true_pos_x']
                y_positions = data[f'uav{i}_true_pos_y']
                plt.plot(x_positions, y_positions, label=f'UAV {i}')
                plt.scatter(x_positions.iloc[0], y_positions.iloc[0], marker='o', label=f'UAV {i} Start')
                plt.scatter(x_positions.iloc[-1], y_positions.iloc[-1], marker='x', label=f'UAV {i} End')

            plt.title('UAV Trajectories from CSV')
            plt.xlabel('X position (m)')
            plt.ylabel('Y position (m)')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            plt.savefig(f'../data/graph/trajectories/uav_trajectories_graph_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.png')
            print(f"Graph successfully saved to uav_trajectories_graph_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.png")
            plt.show()

        except FileNotFoundError:
            print(f"Error: The file {filename} was not found.")
        except Exception as e:
            print(f"An error occurred while plotting: {e}")

    @staticmethod
    def plot_errors_from_csv(filename: str = f'fused_RL_error_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        Plot fusion estimation errors from a CSV file.

        Parameters:
            filename (str): Name of the CSV file to read.
        """

        # Read CSV file
        file_path = f"../data/csv/RL_errors/{filename}"
        data = pd.read_csv(file_path)

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
        ax.set_ylim(0, 200.0)
        ax.legend()
        ax.grid(True)

        # 図4(e)のズームインした図を挿入
        axins = ax.inset_axes([0.5, 0.5, 0.4, 0.4])
        for i in range(2, 7):
             errors = data[f'uav{i}_fused_error']
             valid_times = [t for t, e in zip(data['time'], errors) if e is not None]
             valid_errors = [e for e in errors if e is not None]
             if valid_errors:
                axins.plot(valid_times, valid_errors, color=colors.get(i, 'k'))
        axins.set_xlim(100, 150) # 論文のズーム範囲に合わせる
        axins.set_ylim(0, 0.6)
        axins.grid(True)
        ax.indicate_inset_zoom(axins, edgecolor="black") # ズーム箇所を四角で表示

        plt.show()

