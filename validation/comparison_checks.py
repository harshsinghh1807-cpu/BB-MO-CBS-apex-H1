"""Regression checks for how the benchmark reports cost-vector agreement."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_pex import compare


def result(costs, completed=True):
    return dict(completed=completed, solutions=[dict(cost=c) for c in costs])


class ComparisonChecks(unittest.TestCase):
    def test_exact_front_not_redundant_representatives(self):
        check=compare(result([[1,2,3],[2,3,4]]),result([[1,2,3]]),0)
        self.assertFalse(check['identical_cost_sets'])
        self.assertEqual(check['check'],'PASS')

    def test_approximate_vectors_can_differ(self):
        a,b=result([[100,100,100]]),result([[105,99,102]])
        self.assertEqual(compare(a,b,0.05)['check'],'PASS')
        self.assertEqual(compare(a,b,0)['check'],'FAIL')

    def test_one_way_coverage_not_enough(self):
        self.assertEqual(compare(result([[1,1,1]]),result([[10,10,10]]),0.05)['check'],'FAIL')

    def test_timeout_never_certifies_front(self):
        self.assertEqual(compare(result([],False),result([]),0)['check'],'INCOMPLETE')

    def test_feasibility_mismatch(self):
        self.assertEqual(compare(result([[1,1,1]]),result([]),0.05)['check'],'FAIL')


if __name__=='__main__':
    unittest.main()
