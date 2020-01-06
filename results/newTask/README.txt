All result in this bast package are derived by a feedforward-plus-PI controller

The feedback gains are:
KP = 30
KI = 10

The initial configuration for the cube is:
Tsc_ini = np.array([[1, 0, 0,     1],
					[0, 1, 0,     0.5],
					[0, 0, 1, 0.025],
					[0, 0, 0,     1]])
The goal configuration for the cube is:
Tsc_fin = np.array([[ 0, 1, 0,     0.2],
					[-1, 0, 0,    -2],
					[ 0, 0, 1, 0.025],
					[ 0, 0, 0,     1]])

