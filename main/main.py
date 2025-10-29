import numpy as np
from collections import defaultdict
from typing import List, Dict, Tuple

from quadcopter import UAV, Senario
from estimator import Estimator
from data_handler import Plotter, DataLogger

class MainController:
    """アプリケーション全体を管理し，メインループを実行する"""
    def __init__(self, params: dict):
        self.params = params
        self.uavs: List[UAV] = []
        self.loop_amount: int = 0

        self.estimator = Estimator()
        self.data_logger = DataLogger()

    def initialize(self):
        """システムの初期化"""
        print("ititialize simulation settings...")
        # UAVインスタンス化と初期位置の設定
        initial_positions: dict = self.params['INITIAL_POSITIONS']
        for uav_id, position in initial_positions.items():
            self.uavs.append(UAV(uav_id=uav_id, initial_position=position))
        # デバッグ用のプリント文
        # for uav in self.uavs:
        #     print(uav.id, uav.true_position)

        # 各UAV機の隣接機を設定
        neighbors_setting = self.params['NEIGHBORS']
        for uav in self.uavs:
            if uav.id in neighbors_setting:
                uav.neighbors = neighbors_setting[uav.id]
            #print(uav.neighbors)

        # k=0での直接推定値を設定(直接推定値の初期化)
        # 隣接機に対してのみ初期化
        for uav in self.uavs:
            for neighbor_id in uav.neighbors:
                neighbor_uav = self.uavs[neighbor_id - 1]
                true_initial_rel_pos = neighbor_uav.true_position - uav.true_position
                uav.direct_estimates[neighbor_id] = true_initial_rel_pos.copy()
            #print(uav.direct_estimates)

        self.loop_amount = int(self.params['DURATION'] / self.params['T'])

    def run(self):
        self.initialize()


if __name__ == '__main__':
    simulation_params = {
        'DURATION': 300,
        'T': 0.05,  # サンプリング周期 T
        'GAMMA': 0, # ゲイン γ
        'TARGET_ID': 1, # 推定目標
        'EVENT': Senario.CONTINUOUS, #シナリオ選択
        'INITIAL_POSITIONS': {
            1: [0, 0], 
            2: [2, -30], 
            3: [20, -15],
            4: [-20, 8], 
            5: [-14, 8], 
            6: [-10, -30]
        },
        'NEIGHBORS': { 
            1: [],         #UAV1の隣接機(推定対象のため無し)
            2: [1],        #UAV2の隣接機        
            3: [1, 4, 5],  #UAV3の隣接機      
            4: [1],        #UAV4の隣接機
            5: [3, 4],     #UAV5の隣接機
            6: [4]         #UAV6の隣接機
        },
        'NOISE': { 
            'delta_bar': 0.5,
            'dist_bound': 0.05
        }
    }

    controller = MainController(simulation_params)
    controller.run()
