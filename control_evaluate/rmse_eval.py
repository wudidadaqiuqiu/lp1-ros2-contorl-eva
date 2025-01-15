import numpy as np

class RmseEval:
    def __init__(self):
        self.temp = 0
        self.cnt = 0
        self.ref = None
    def evaluate(self, one_step_fdb, one_step_ref = None):
        if (one_step_ref is not None):
            self.ref = one_step_ref
        if self.ref is None:
            return None
        self.temp += np.square(self.ref - one_step_fdb)
        self.cnt += 1
        return np.sqrt(self.temp / self.cnt)

    @staticmethod
    def get_name():
        return "rmse"

eval_types = {
    RmseEval.get_name(): RmseEval
}