import numpy as np

from guilda.branch.branch import Branch

class BranchPi(Branch):
    """ モデル ：対地静電容量をもつ送電線のπ型回路モデル
親クラス：branchクラス
実行方法：obj = branch_pi(from, to, x, y)
 引数 ：・from,to : 接続する母線番号
     ・ x ：[1*2 double]の配列。インピーダンスの実部、虚部を並べた配列。
     ・ y ：double値。対地静電容量の値
 出力 ：branchクラスのインスタンス

restrictions: SetAccess = public
    """

    def __init__(self, from_, to, x, y):
        super().__init__(from_, to)

        if type(x) == list:
            x = complex(x[0], x[1])
        self.x = x
        self.y = y

    def get_admittance_matrix(self):
        x = self.x
        y = self.y
        Y = np.array(([complex(1/x,y), -1/x          ],
                      [-1/x          , complex(1/x,y)]))
        return Y
