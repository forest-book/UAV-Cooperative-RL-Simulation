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
        self.dt = params['T']   # サンプリング周期
        self.event = params['EVENT']    # t=100sで外乱を加えるか否か

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

        # k=0での融合推定値を設定(融合推定値の初期化)
        # UAV_i(i=2~6)から見たUAV1の相対位置を融合推定
        target_id = self.params['TARGET_ID']
        for i in range(1,6):
            true_initial_rel_pos: np.ndarray = self.uavs[target_id - 1].true_position - self.uavs[i].true_position
            self.uavs[i].fused_estimates[target_id] = true_initial_rel_pos.copy()
            #print(self.uavs[i].fused_estimates)

        # 推定式はステップk(自然数)毎に状態を更新するため
        self.loop_amount = int(self.params['DURATION'] / self.params['T'])
        #print(f"step num: {self.loop_amount}")

    def get_noisy_measurements(self, uav_i: UAV, uav_j: UAV) -> Tuple[np.ndarray, float, float]:
        """
        2UAV間の真の状態に基づき、ノイズが付加された測定値を生成する
        シミュレータ上に測距モジュールがあるなら不要となる関数
        """
        # 真の相対値
        true_x_ij = uav_j.true_position - uav_i.true_position
        true_v_ij = uav_j.true_velocity - uav_i.true_velocity # 自機の速度情報はUWB RCM通信で送られてくる
        true_d_ij = np.linalg.norm(true_x_ij) # UWBモジュールでの測距を模している
        # 論文式(1)の上あたりの方程式から算出される
        true_d_dot_ij = (true_x_ij @ true_v_ij) / (true_d_ij + 1e-9) # ゼロ除算防止

        # ノイズモデル (4.1節)
        delta_bar = self.params['NOISE']['delta_bar']
        dist_bound = self.params['NOISE']['dist_bound']
        
        # 速度ノイズ: [-δ̄/2, δ̄/2] の一様乱数
        vel_noise = np.random.uniform(-delta_bar / 2, delta_bar / 2, size=2)
        # 距離ノイズ: [-bound/2, bound/2] の一様乱数
        dist_noise = np.random.uniform(-dist_bound / 2, dist_bound / 2)
        # 距離変化率ノイズ: [-bound/2, bound/2] の一様乱数
        dist_rate_noise = np.random.uniform(-dist_bound / 2, dist_bound / 2)

        return true_v_ij, true_d_ij, true_d_dot_ij

    def run(self):
        """メインループの実行"""
        self.initialize()

        #for loop in self.loop_amount:
        for loop in range(1): #5ループでのデバッグ用
            
            # 直接推定の実行
            for uav_i in self.uavs:
                # 各UAVが自機のすべての隣接機に対して行う
                print(f"uav_{uav_i.id}")
                for neighbor_id in uav_i.neighbors:
                    neighbor_uav = self.uavs[neighbor_id - 1]
                    print(f"uav_{uav_i.id}_{neighbor_id}")
                    # ノイズ付き観測値を取得
                    noisy_v, noisy_d, noisy_d_dot = self.get_noisy_measurements(uav_i, neighbor_uav)
                    print(f"相対速度: {noisy_v}")
                    print(f"距離: {noisy_d}")
                    print(f"距離の変化率: {noisy_d_dot}")
                    # 式(1)の計算
                    next_direct = self.estimator.calc_direct_RL_estimate(
                        chi_hat_ij_i_k=uav_i.direct_estimates[neighbor_id],
                        noisy_v=noisy_v,
                        noisy_d=noisy_d,
                        noisy_d_dot=noisy_d_dot,
                        T=self.dt,
                        gamma=self.params['GAMMA']
                    )
                    print(f"直接推定値: {next_direct}")
                    print("-"*50)
                print("="*50)






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
