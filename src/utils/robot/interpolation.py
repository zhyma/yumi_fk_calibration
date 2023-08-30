#!/usr/bin/env python3

import matplotlib.pyplot as plt

import numpy as np
from copy import deepcopy

def interpolation(points, n_samples, dt):
    '''
    Input:
        - a n*7 list [[q1, q2, q7, q3, q4, q5, q6],[q1', ..., q6'], ...]
        - the number of samples bewteen two given point
        - dt: 
    Output: 
        - a (n+2)*7 list [[q1, q2, q7, q3, q4, q5, q6], [q1i1, q2i1, ...], ...]
    '''
    q = points
    qd = [[0]*7]
    j_val = deepcopy([q[0]])

    if len(q) <= 2:
        ## only the start and the end is given, interpolate evenly
        # j_interval = [ [0]*7 for _ in range(n_samples)]
        # the increasemental value of each joint
        dj = [(q[-1][i]-q[0][i])/(n_samples+1) for i in range(7)]
        # print("dj is: ")
        # for i in dj:
        #     print(i)
        for i in range(n_samples):
            val = [q[0][j] + dj[j]*(i+1) for j in range(7)]
            j_val.append(val)

        j_val.append(q[-1])


    else:
        ## Generate velocity at the knot points
        ## Using simple heuristic (average)
        ## 0.5*((q(t)-q(t-1))/dt + (q(t+1)-q(t))/dt)
        ## starts and ends with all 0
        for i in range(1, len(q)-1):
            qdi = []
            for j in range(7):
                ## q_t_minus_1
                qm1 = q[i-1][j]
                ## q_t, current q
                qt = q[i][j]
                ## q_t_plus_1
                qp1 = q[i+1][j]

                if (qm1 < qt and qt < qp1) or (qm1 > qt and qt > qp1):
                    qdi.append(0.5*((qt-qm1)/dt+(qp1-qt)/dt))
                else:
                    qdi.append(0)

            qd.append(qdi)

        qd.append([0]*7)

        ## calculate the coefficient a0, a1, a2, and a3
        si = q[0]
        sdi = qd[0]
        for i in range(1, len(q)):
            ## j_interval = [[0]*7]*n_samples # all [0]*7 are the same instance, change one will change all others
            j_interval = [ [0]*7 for _ in range(n_samples)]
            gi = q[i]
            gdi = qd[i]
            for j in range(7):
                ## a0 = si
                a0 = si[j]
                ## a1 = sd(i)
                a1 = sdi[j]
                ## a2 = (gi-si)*3/(T^2)-(sdi)*2/T-gdi/T
                a2 = (gi[j]-si[j])*3/(dt**2)-(sdi[j])*2/dt-gdi[j]/dt
                ## a3 = -(gi-si)*2/(T^3)+(sdi+gdi)/(T^2)
                a3 = -(gi[j]-si[j])*2/(dt**3)+(sdi[j]+gdi[j])/(dt**2)
                for k in range(n_samples):
                    ## q(t) = a0+a1*t+a2*t^2+a3*t^3
                    t = dt/(n_samples+1) * (k+1)
                    j_interval[k][j] = a0+a1*t+a2*t**2+a3*t**3

            for val in j_interval:
                j_val.append(val)

            j_val.append(q[i])

            si = q[i]
            sdi = qd[i]

    return j_val

if __name__ == '__main__':
    jval_list = [[-1.910732488289212, -1.948345991647264, 0.9311644675113045, -0.48049726173310014, 1.0077526698255987, 1.4434956391402247, 3.7831853071795862],\
            [-1.4381221301992597, -1.969164396057361, 1.4351484047723992, 0.11691985332275436, 0.793984982122215, 1.1115103530485912, 3.2595865315812875],\
            [-1.0759694790727565, -1.7653101852953867, 1.7330118510252739, 0.6222073599452446, 0.6174048532482165, 0.8363342378308655, 2.735987755982989],\
            [-0.7884360875854216, -1.545870013186837, 1.7167676760784707, 1.0409357686695022, 0.43614746066227267, 0.6997857874680693, 2.2123889803846897],\
            [-0.5803742986909544, -1.4235881376233823, 1.3166080150070254, 1.250989901676155, 0.07649707956166046, 0.7033261326647234, 1.688790204786391],\
            [-0.6046062436360865, -1.2980922735887068, 0.8880908456623958, 1.1317601651623392, -0.28510848805083666, 0.7348967843386524, 1.1651914291880923],\
            [-0.7699325682092536, -1.0985972697380182, 0.8290117110734384, 0.8059631420484361, -0.359012707204586, 0.8009826103317225, 0.6415926535897931],\
            [-1.0006462964857854, -0.8955528021482563, 1.0285749923067173, 0.4131108853818319, -0.38690927379367845, 0.9400216063591692, 0.11799387799149441],\
            [-1.3107402450046948, -0.70900169540075, 1.444257079718732, 0.01732641434199542, -0.524982263813518, 1.1450163134304823, -0.4056048976068043],\
            # [1.3029014885683399, 0.6220446189990329, -0.9742496391846671, -0.50153917321161, -0.7714379549658814, 1.471140075463862, -0.9292036732051034],\
            [-1.7176571851906388, -1.0712154452098304, 1.6579703902086416, -0.36542419623602856, -0.15849570121684894, 1.2137938528221757, -1.4528024488034017],\
            [-1.6462398604161814, -1.4008239170183066, 1.3920098862491554, -0.20046380540953238, 0.28308688057483444, 1.1438220560595516, -1.9764012244017009]]


    j_list = interpolation(jval_list, 10, 2)

    l = len(j_list)
    x = list(range(l))

    for j in range(7):
        plt.plot(x, [i[j] for i in j_list])
    plt.show()
