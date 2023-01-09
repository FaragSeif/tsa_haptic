import numpy as np

class RecursiveNamespace: # without extending SimpleNamespace!

  @staticmethod
  def map_entry(entry):
    if isinstance(entry, dict):
      return RecursiveNamespace(**entry)

    return entry

  def __init__(self, **kwargs):
    for key, val in kwargs.items():
      if type(val) == dict:
        setattr(self, key, RecursiveNamespace(**val))
      elif type(val) == list:
        setattr(self, key, list(map(self.map_entry, val)))
      else: # this is the only addition
        setattr(self, key, val)
        
        

def fit_quadratic(x, y):
    
    A = np.array([x**2, x, np.ones(len(x))]).T
    y = np.array(y)
    coefficients = np.linalg.pinv(A)@y
    a, b, c = coefficients
    x_0 = -b/(2*a)
    y_0 = a*x_0**2 + b*x_0 + c
    
    return x_0, y_0, a, b, c
