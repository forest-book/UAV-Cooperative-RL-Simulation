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

        self.estimator = Estimator()
        self.data_logger = DataLogger()

    def initialize(self):
        print("init")

    def run(self):
        print("hollo")


if __name__ == '__main__':
    simulation_params = {
        'DURATION': 300,
        'T': 0.05,  # サンプリング周期 T
        'GAMMA': 0, # ゲイン γ
        'TARGET_ID': 1, # 推定目標
        'EVENT': Senario.CONTINUOUS, #シナリオ選択
        'INITIAL_POSITIONS': {
            1: [0, 0], 2: [2, -30], 3: [20, -15],
            4: [-20, 8], 5: [-14, 8], 6: [-10, -30]
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
