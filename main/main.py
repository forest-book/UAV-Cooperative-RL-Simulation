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
                uav.direct_estimates[f"chi_{uav.id}_{neighbor_id}"].append(true_initial_rel_pos.copy())
            #print(uav.direct_estimates)

        # k=0での融合推定値を設定(融合推定値の初期化)
        # UAV_i(i=2~6)から見たUAV1の相対位置を融合推定
        target_id = self.params['TARGET_ID']
        for i in range(1,6):
            true_initial_rel_pos: np.ndarray = self.uavs[target_id - 1].true_position - self.uavs[i].true_position
            self.uavs[i].fused_estimates[f"pi_{self.uavs[i].id}_{target_id}"].append(true_initial_rel_pos.copy())
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

        return true_v_ij + vel_noise, true_d_ij + dist_noise, true_d_dot_ij + dist_rate_noise

    def calc_RL_estimation_error(self, uav_i_id, target_j_id, loop_num):
        #print(f"uav_{uav_i_id}の誤差計算")
        true_rel_pos = self.uavs[target_j_id - 1].true_position - self.uavs[uav_i_id - 1].true_position
        #print(f"真の相対位置: {true_rel_pos}")
        estimate_rel_pos = self.uavs[uav_i_id - 1].fused_estimates[f"pi_{uav_i_id}_{target_j_id}"]
        #print(estimate_rel_pos)
        estimation_error = estimate_rel_pos[loop_num] - true_rel_pos
        #print(f"推定誤差: {estimation_error}")
        estimation_error_distance = np.linalg.norm(estimation_error)
        #print(f"推定誤差の距離: {estimation_error_distance}")
        return estimation_error_distance
    
    def show_simulation_progress(self, loop):
        if(loop * 100 // self.loop_amount) > ((loop - 1) *100 // self.loop_amount):
            print(f"simulation progress: {loop *100 // self.loop_amount}%")

    def run(self):
        """メインループの実行"""
        self.initialize()

        for loop in range(self.loop_amount):
        #for loop in range(150): #5ループでのデバッグ用
            #print(f"***** sim step {loop + 1} *****")
            # 1.直接推定の実行
            for uav_i in self.uavs:
                #print(f"uav_{uav_i.id}")
                for neighbor_id in uav_i.neighbors:
                    neighbor_uav = self.uavs[neighbor_id - 1]
                    #print(f"uav_{uav_i.id}_{neighbor_id}")
                    #print(f"uav_{uav_i.id}の速度: {uav_i.true_velocity}")
                    #print(f"uav_{neighbor_id}の速度: {neighbor_uav.true_velocity}")
                    # ノイズ付き観測値を取得
                    noisy_v, noisy_d, noisy_d_dot = self.get_noisy_measurements(uav_i, neighbor_uav)
                    #print(f"相対速度: {noisy_v}")
                    #print(f"距離: {noisy_d}")
                    #print(f"距離の変化率: {noisy_d_dot}")
                    # 式(1)の計算
                    chi_hat_ij_i_k = uav_i.direct_estimates[f"chi_{uav_i.id}_{neighbor_id}"] # k=loopの時の直接推定値を持ってくる
                    #print(f"前ステップの相対位置: {chi_hat_ij_i_k[loop]}")
                    next_direct = self.estimator.calc_direct_RL_estimate(
                        chi_hat_ij_i_k=chi_hat_ij_i_k[loop],
                        noisy_v=noisy_v,
                        noisy_d=noisy_d,
                        noisy_d_dot=noisy_d_dot,
                        T=self.dt,
                        gamma=self.params['GAMMA']
                    ) # 次のステップ(k=loop + 1)の時の相対位置を直接推定
                    #print(f"直接推定値: {next_direct}")
                    # uav_iは直接推定値を持っている
                    uav_i.direct_estimates[f"chi_{uav_i.id}_{neighbor_id}"].append(next_direct.copy())
                    
                    #print("-"*50)
                
                #print("="*50)

            # 2.融合推定の実行
            #print("融合推定の実行")
            # UAV_i(i=2~6)がUAV_1への融合推定値を算出する
            target_j_id = self.params.get('TARGET_ID')
            target_j_uav: UAV = self.uavs[target_j_id - 1]
            for uav_i in self.uavs:
                if uav_i.id == target_j_id:
                    continue # UAV1 (j=1) は自身への推定を行わない
                #print(f"uav_{uav_i.id}")
                # 重みκを計算
                kappa_D, kappa_I = self.estimator.calc_estimation_kappa(uav_i.neighbors.copy(), target_j_id) # Listは参照渡しなのでcopyを渡す
                # print(f"kappa_D: {kappa_D}")
                # print(f"kappa_I: {kappa_I}")

                # ノイズ付き相対速度 v_ij を取得
                noisy_v_ij, _, _ = self.get_noisy_measurements(uav_i, target_j_uav)
                # print(f"uav_{uav_i.id}の速度: {uav_i.true_velocity}")
                # print(f"target uavの速度: {target_j_uav.true_velocity}")
                # print(f"相対速度: {noisy_v_ij}")

                # 直接推定値と融合推定値を持ってくる
                chi_hat_ij_i_k = uav_i.direct_estimates[f"chi_{uav_i.id}_{target_j_id}"] # k=loopの時の直接推定値を持ってくる
                pi_ij_i_k = uav_i.fused_estimates[f"pi_{uav_i.id}_{target_j_id}"]
                #print(pi_ij_i_k)

                # 間接推定値のリストを作成
                indirect_estimates_list: List = []
                for r_id in uav_i.neighbors:
                    if r_id == target_j_id: # r(間接機)はtarget(推定対象)であってはならない
                        continue

                    uav_r = self.uavs[r_id - 1] #uav_iの隣接機UAVオブジェクト
                    
                    # uav_i(自機)からuav_r(間接機)への直接推定値
                    chi_hat_ir_i_k = uav_i.direct_estimates[f"chi_{uav_i.id}_{uav_r.id}"]
                    # print(f"chi_hat_{uav_i.id}_{uav_r.id}: {chi_hat_ir_i_k}")
                    # uav_r(間接機)からtarget(推定対象)への融合推定値
                    pi_rj_r_k = uav_r.fused_estimates[f"pi_{uav_r.id}_{target_j_id}"]
                    # print(f"pi_{uav_r.id}_{target_j_id}: {pi_rj_r_k}")
                    # uav_i(自機)からtarget(推定対象)への間接推定値
                    chi_hat_ij_r_k: np.ndarray = chi_hat_ir_i_k[loop] + pi_rj_r_k[loop]
                    #print(f"間接推定値: {chi_hat_ij_r_k}")
                    indirect_estimates_list.append(chi_hat_ij_r_k.copy())
                    #print("*"*50)
                
                #print(f"前ステップの融合推定位置: {pi_ij_i_k[loop]}")
                next_fused = self.estimator.calc_fused_RL_estimate(
                    pi_ij_i_k=pi_ij_i_k[loop],
                    direct_estimate_x_hat=chi_hat_ij_i_k[loop] if kappa_D!=0 else np.zeros(2),
                    indirect_estimates=indirect_estimates_list,
                    noisy_v=noisy_v_ij,
                    T=self.dt,
                    kappa_D=kappa_D,
                    kappa_I=kappa_I
                )
                #print(f"融合推定値: {next_fused}") # k+1の時の値
                uav_i.fused_estimates[f"pi_{uav_i.id}_{target_j_id}"].append(next_fused.copy())
                #print("#"*50)

            # 結果をlogに保存する（update_state前の位置を記録）
            self.data_logger.logging_timestamp(loop * self.dt)
            #print(f"時間: {loop*self.dt}")

            # 全UAVの軌道を記録（現在の位置 k を記録）
            for uav in self.uavs:
                self.data_logger.logging_uav_trajectories(uav_id=uav.id, uav_position=uav.true_position.copy())

            # 全UAVの真の状態を k+1 に更新
            for uav in self.uavs:
                uav.update_state(t=loop+1, dt=self.dt, event=self.params['EVENT'])

            for uav in self.uavs:
                if uav.id == self.params['TARGET_ID']:
                    continue
                # k+1時点での推定誤差を計算
                error_distance = self.calc_RL_estimation_error(uav.id, self.params['TARGET_ID'], loop+1)
                # 推定誤差をロギング
                self.data_logger.logging_fused_RL_error(uav_id=uav.id, error=error_distance)

            self.show_simulation_progress(loop=loop)

        # ロギングした推定誤差をcsv出力
        self.data_logger.save_fused_RL_errors_to_csv()
        self.data_logger.save_trajectories_data_to_csv()
        Plotter.plot_trajectories_from_csv()
        Plotter.plot_errors_from_csv()
        self.data_logger.print_fused_RL_error_statistics(transient_time=10.0)
        self.data_logger.save_fused_RL_error_statistics(transient_time=10.0)
        self.data_logger.save_fused_RL_error_statistics(transient_time=10.0, format='txt')

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
