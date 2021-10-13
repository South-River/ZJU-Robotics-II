import math
import numpy


def fact(n):
    if(n == 0):
        return 1
    else:
        return n * fact(n - 1)

class MinSnapCloseForm():
    def init(self, waypoints, meanvel):
        self.n_order = 7
        self.waypoints = waypoints
        self.n_seg = int(len(waypoints) - 1)
        self.meanvel = meanvel
        self.start_condition = numpy.zeros(4,0)
        self.start_condition[0,0] = self.waypoints[0]
        self.end_condition = numpy.zeros(4,0)
        self.end_condition[0,0] = self.waypoints[-1]
        self.ts = numpy.zeros(self.n_seg, 1)
        self.Q = numpy.zeros(self.n_seg * (self.n_order + 1), self.n_seg * (self.n_order + 1))
        self.M = numpy.zeros(self.n_seg * (self.n_order + 1), self.n_seg * (self.n_order + 1))
        self.Ct = numpy.zeros(self.n_seg * (self.n_order + 1), 4 * (self.n_seg + 1))

    def block(self, matrix_A, matrix_B, start_i, start_j, matrix_B_col, matrix_B_row):
        for i in range(matrix_B_row):
            for j in range(matrix_B_col):
                matrix_A[start_j + j][start_i + i] = matrix_B[j][i]
        return matrix_B

    def calQ(self):
        self.Q = numpy.zeros(self.n_seg * (self.n_order + 1), self.n_seg * (self.n_order + 1))

        for k in range(0, self.n_seg + 1):
            Q_k = numpy.zeros(self.n_order + 1, self.n_order + 1)
            for i in range(4, self.n_order + 1):
                for j in range(4, self.n_order + 1):
                    Q_k[i, j] = (fact(i) / fact(i - 4)) * (fact(j) / fact(j - 4)) / (i + j - 7) * pow(self.ts[k][0], (i + j - 7))
            self.Q = self.block(Q, Q_k, k * (self.n_order + 1), k * (self.n_order + 1), self.n_order + 1, self.n_order + 1)

    def calM(self):
        self.M = numpy.zeros(self.n_seg * (self.n_order + 1), self.n_seg * (self.n_order + 1))

        for k in range(0, self.n_seg + 1):
            M_k = numpy.zeros(self.n_order + 1, self.n_order + 1)

            for i in range(0, 4):
                M_k[i,i] = fact(i)

            for i in range(0, self.n_order + 1):
                for j in range(0, 4):
                    if i >= j:
                        M_k[j+4][i] = fact(i) / fact(i - j) * pow(self.ts[k][0], i -j)
            self.M = self.block(self.M, M_k, k * (self.n_order + 1), k * (self.n_order + 1), self.n_order + 1, self.n_order + 1)

    def calCT(self):
        self.Ct = numpy.zeros(self.n_seg * (self.n_order + 1), 4 * (self.n_seg + 1))

        row = self.n_seg * (self.n_order + 1)
        col = 4 * (self.n_seg + 1)

        for i in range(0, self.n_seg + 1):
            self.Ct = self.block(self.Ct, numpy.identity(self.n_order + 1), i * (self.n_order + 1), i * 4, self.n_order + 1, self.n_order + 1)

        self.Ct_dF = numpy.zeros(row, 8 + (self.n_seg - 1))
        self.Ct_dP = numpy.zeros(row, (self.n_seg - 1) * 3)

        self.Ct_dF[:,0:3] = self.Ct[:,0:3]

        for i in range(self.n_seg-1):
            self.Ct_dF[:,i+4] = self.Ct[:,i+1]
            self.Ct_dP[:,i*3+1:i*3+3] = self.Ct_dP[:,i*4+6:i*4+8]
        self.Ct_dF[:,-4:-1] = self.Ct[:,-4:-1]
        self.Ct[:,0:8+(self.n_seg-1)-1] = self.Ct_dF
        self.Ct[:,8+(self.n_seg-1):4*(self.n_seg+1)] = self.Ct_dP

    def minSnapCloseFormServer(self):
        self.calQ()
        self.calM()
        self.calCT()
        R = self.Ct.transpose() * numpy.self.M.inv().transpose() * self.Q * numpy.self.M.inv() * self.Ct
        R_pp = R[-3*(self.n_seg-1):-1,-3*(self.n_seg-1):-1]
        R_fp = R[0:self.n_seg+7-1,-3*(self.n_seg-1):-1]
        dF = numpy.zeros(self.n_seg+7,1)
        dF[0:3,] = self.start_condition[0:3,]
        for i in range(1, self.n_seg):
            dF[i+3, 0] = self.waypoints[i]
        dF[self.n_seg + 3:self.n_seg+6,:] = self.end_condition[0:3,]
        dP = -R_pp.inv() * R_fp.transpose() * dF
        d = numpy.zeros(self.n_seg * (self.n_order + 1), 1)
        d[0:self.n_seg+7-1,]=dF[0:self.n_seg+7-1,]
        d[self.n_seg+7:self.n_seg*(self.n_order+1)-1,]=dP[0:self.n_seg*(self.n_order+1)-(self.n_seg+7),]

        dec_vel = self.Ct * d

        poly_coef = numpy.self.M.inv() * dec_vel

        return poly_coef,dec_vel
