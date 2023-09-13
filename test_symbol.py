import guilda.backend as G
from guilda.models.sample import simple_3_bus

G.set_config('torch')

print(G.get_config())

print(G.array([1,23,4]).flatten())

print(G.cvec([]).shape)

print(G.rvec([]).shape)

print(G.pi)

net = simple_3_bus()
net.initialize()

net.print_bus_state()