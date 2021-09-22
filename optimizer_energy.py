# -*- coding utf-8 -*-
'''
#############chh###########
测试能耗问题是否有优化空间
#########################
'''
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
import numpy as np
from scipy.optimize import minimize
from scipy.optimize import least_squares

def op_fun_max(args):
    k_t = args
    fun = lambda x: k_t*np.max([np.linalg.norm(np.array([x[0],x[1],x[2]]))**1.5, np.linalg.norm(np.array([x[3],x[4],x[5]]))**1.5, np.linalg.norm(np.array([x[6],x[7],x[8]]))**1.5])
    return fun
def op_fun_sum(args):
    k_t = args
    fun = lambda x: k_t*np.sum([np.linalg.norm(np.array([x[0],x[1],x[2]]))**1.5, np.linalg.norm(np.array([x[3],x[4],x[5]]))**1.5, np.linalg.norm(np.array([x[6],x[7],x[8]]))**1.5])
    return fun
def con(args):
    #作用点：顶点，质心点,重力
    triangle_angle_1, triangle_angle_2,triangle_angle_3,payload_center,payload_gravity= args
    # 公式12
    constrants = (
        {'type':'eq','fun':lambda x: (np.cross(triangle_angle_1,np.array([x[0],x[1],x[2]]))+np.cross(triangle_angle_2,np.array([x[3],x[4],x[5]]))+np.cross(triangle_angle_3,np.array([x[6],x[7],x[8]]))+np.cross(payload_center,payload_gravity))[0]},
        {'type':'eq','fun':lambda x: (np.cross(triangle_angle_1,np.array([x[0],x[1],x[2]]))+np.cross(triangle_angle_2,np.array([x[3],x[4],x[5]]))+np.cross(triangle_angle_3,np.array([x[6],x[7],x[8]]))+np.cross(payload_center,payload_gravity))[1]},
        {'type':'eq','fun':lambda x: (np.cross(triangle_angle_1,np.array([x[0],x[1],x[2]]))+np.cross(triangle_angle_2,np.array([x[3],x[4],x[5]]))+np.cross(triangle_angle_3,np.array([x[6],x[7],x[8]]))+np.cross(payload_center,payload_gravity))[2]},
        # {'type':'eq','fun':lambda x: triangle_angle_1[1]*x[2]-x[1]*triangle_angle_1[2]+triangle_angle_2[1]*x[5]-x[4]*triangle_angle_2[2]+triangle_angle_3[1]*x[8]-x[7]*triangle_angle_3[2]+payload_center[1]*payload_gravity[2]-payload_center[2]*payload_gravity[1]},
        # {'type':'eq','fun':lambda x: triangle_angle_1[2]*x[0]-x[2]*triangle_angle_1[0]+triangle_angle_2[2]*x[3]-x[5]*triangle_angle_2[0]+triangle_angle_3[2]*x[6]-x[8]*triangle_angle_3[0]+payload_center[2]*payload_gravity[0]-payload_center[0]*payload_gravity[2]},
        # {'type':'eq','fun':lambda x: triangle_angle_1[0]*x[1]-x[0]*triangle_angle_1[1]+triangle_angle_2[0]*x[4]-x[3]*triangle_angle_2[1]+triangle_angle_3[0]*x[7]-x[6]*triangle_angle_3[1]+payload_center[0]*payload_gravity[1]-payload_center[1]*payload_gravity[0]},                
                    {'type':'eq','fun':lambda x: x[0]+x[3]+x[6]+payload_gravity[0]},
                    {'type':'eq','fun':lambda x: x[1]+x[4]+x[7]+payload_gravity[1]},
                    {'type':'eq','fun':lambda x: x[2]+x[5]+x[8]+payload_gravity[2]}
                    )
    return constrants



def minimizeForce(allArgs):
    triangle_angle_1, triangle_angle_2,triangle_angle_3,payload_center,payload_gravity,k_t= allArgs
    args = [triangle_angle_1, triangle_angle_2,triangle_angle_3,payload_center,payload_gravity]
    cons = con(args)

    x0 = np.asarray([0,0,1/3, 0,0,1/3,0,0,1/3])
    # print(x0[0][0])
    res_max = minimize(op_fun_max(k_t), x0, method='SLSQP', constraints=cons)
    res_sum = minimize(op_fun_sum(k_t), x0, method='SLSQP', constraints=cons)
    v_max = res_max.x  # 返回最小化后的变量
    v_sum = res_sum.x  # 返回最小化后的变量
    result_max = res_max.fun # 返回最小化后的结果
    result_sum = res_sum.fun # 返回最小化后的结果
    # print('result',result)
    # print(res_max.x)
    # print(res_sum.x)
    print(res_max.success)
    print(res_sum.success)
    return v_max,v_sum,result_max,result_sum

if __name__ == "__main__":
    # print(np.linalg.norm(np.array([1,2,3]))**1.5)
    import time
    s = time.time()
    # args = 作用点1，作用点2，作用点3，质心位置，重力向量，k值
    payload_pos = np.array([0,3/4*np.sqrt(3),1])
    payload_gravity = np.array([1,0,-1])
    ''' 
    固定作用点：np.array([0,2/3*np.sqrt(3),1])
    [1,0,-1]:delta:0.628-0.48=0.148===>
    [1,1,-1]:delta:0.55-0.43=0.12==>
    [1,-1,-1]=:delta::0.55-0.43=0.12==>
    [0,-1,-1]:delat:0.56-0.47=0.09
    [0,1,-1]:delta:0.56-0.47=0.09
    '''
    '''
    固定力向量：【1，0，-1】,max的3个模很接近
    np.array([0,3/4*np.sqrt(3),1])==>delta:0.7-0.53=0.17
    np.array([0,5/6*np.sqrt(3),1])==>delta:0.78-0.59=0.19
    np.array([0,6/7*np.sqrt(3),1])==>delta:0.8-0.6=0.2
    np.array([0,1*np.sqrt(3),1]) ==>delata:0.93-0.7 = 0.23

    '''
    payload_gravity = payload_gravity/np.linalg.norm(payload_gravity)
    args = (np.array([1,0,1]), np.array([-1,0,1]), np.array([0,np.sqrt(3),1]), payload_pos,payload_gravity ,4.328)
    # print(np.array([0,0,-1])[2])
    v_max,v_sum,result_max,result_sum  = minimizeForce(args)
    # print(v_max)
    sum_norm_1,sum_norm_2,sum_norm_3 =np.linalg.norm(v_sum[0:3]),np.linalg.norm(v_sum[3:6]),np.linalg.norm(v_sum[6:9])
    max_norm_1,max_norm_2,max_norm_3 =np.linalg.norm(v_max[0:3]),np.linalg.norm(v_max[3:6]),np.linalg.norm(v_max[6:9])
    print('force_sum_1',v_sum[0:3],'<==>','force_max_1',v_max[0:3])
    print('force_sum_2',v_sum[3:6],'<==>','force_max_2',v_max[3:6])
    print('force_sum_3',v_sum[6:9],'<==>','force_max_3',v_max[6:9])
    print('sum_1: ',sum_norm_1,'<==>','max_1: ',max_norm_1)
    print('sum_2: ',sum_norm_2,'<==>','max_2: ',max_norm_2)
    print('sum_3: ',sum_norm_3,'<==>','max_3: ',max_norm_3)

