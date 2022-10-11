using HIPPO
using POMDPs

m = TargetSearchPOMDP()

transition(m, TSState([1,1], [10,10]), :left)