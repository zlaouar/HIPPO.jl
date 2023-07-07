using HIPPO
using POMDPs
using Test

rewarddist = [3.0 3.0;
              3.0 3.0]
running_cost = -1.0
ns = length(rewarddist)
mapsize = (2,2)
sinit = TSState([1,1],[3,3],trues(prod(mapsize)))

m = TargetSearchPOMDP(sinit, size=mapsize, rewarddist=rewarddist)
s = sinit
sp = TSState([2,1],[3,3],trues(prod(mapsize)))
a = :right
function getind(pos)
    correct_ind = reverse(pos)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]] 
    return inds
end

@testset "reward" begin
    running_cost = -1.0
    @test reward(m, s, a, sp) == running_cost + rewarddist[getind(sp.robot)...]
end

sts = unique(getfield.(states(m), :robot))[1:end-1]
acs = (:right, :up, :left, :down)
rtotal = 0.0

@testset "rtotal" begin
    function rtotaltest(rtotal,s,acs)
        for i ∈ 1:ns
            a = acs[i]
            sp = rand(transition(m, s, a))
            rtotal += reward(m, s, a, sp)
            #println(reward(m, s, a, sp), ", a:", a)
            println("s: ", s.visited, " | sp: ", sp.visited)
            s = sp
        end
        return rtotal
    end
    @test rtotaltest(rtotal,s,acs) == sum(rewarddist) + ns*running_cost - rewarddist[getind(sinit.robot)...]
end
