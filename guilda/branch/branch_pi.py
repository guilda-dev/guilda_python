import numpy as np

from guilda.branch.branch import Branch


from guilda.utils.typing import ComplexArray

class BranchPi(Branch):
    ''' モデル：対地静電容量をもつ送電線のπ型回路モデル
親クラス：branchクラス
実行方法：obj = branch_pi(from, to, x, y)
    引数：・from,to : 接続する母線番号
     ・ z：complex。インピーダンス。
     ・ y：double値。対地静電容量の値
    出力：branchクラスのインスタンス

restrictions: SetAccess = public
    '''

    def __init__(self, from_: int, to: int, z: complex, y: float):
        super().__init__(from_, to)

        self.z: complex = z
        self.y: float = y

    def get_admittance_matrix(self) -> ComplexArray:
        z = self.z
        y = self.y
        Y = np.array([
            [complex(1/z,y), -1/z          ],
            [-1/z          , complex(1/z,y)]
        ])
        return Y
