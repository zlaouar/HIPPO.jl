@testset "reward" begin
    rewarddist = [3.0 3.0;
                3.0 3.0]
    running_cost = -1.0
    ns = length(rewarddist)
    mapsize = (2,2)
    sinit = RewardState([1,1],[3,3],trues(prod(mapsize)))

    m = RewardPOMDP(sinit, size=mapsize, rewarddist=rewarddist)
    s = sinit
    sp = RewardState([2,1],[3,3],trues(prod(mapsize)))
    a = :right

    running_cost = -1.0
    @test reward(m, s, a, sp) == running_cost + rewarddist[getind(sp.robot)...]
end
