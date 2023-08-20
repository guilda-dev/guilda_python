import guilda.backend as G

G.set_config('numpy')

print(G.get_config())

print(G.array([1,23,4]).flatten())

print(G.cvec([]).shape)

print(G.rvec([]).shape)

print(G.pi)