@testset "rtotal" begin
    rewarddist = [3.0 3.0;
                3.0 3.0]
    running_cost = -1.0
    ns = length(rewarddist)
    mapsize = (2,2)
    sinit = RewardState([1,1],[3,3],trues(prod(mapsize)))

    m = RewardPOMDP(sinit, size=mapsize, rewarddist=rewarddist)
    s = sinit

    function getind(pos)
        correct_ind = reverse(pos)
        xind = m.size[2]+1 - correct_ind[1]
        inds = [xind, correct_ind[2]] 
        return inds
    end

    acs = (:right, :up, :left, :down, :right)
    acs_len = length(acs)
    rtotal = 0.0
    r = 0.0

    function rtotaltest(m,rtotal,s,acs)
        for i ∈ 1:acs_len
            a = acs[i]
            sp = rand(transition(m, s, a))
            r = reward(m, s, a, sp)
            rtotal += r
            #println(reward(m, s, a, sp), ", a:", a)
            #println("s: ", s.robot, " -- ", s.visited, " | sp: ", sp.robot, " -- ", sp.visited, " | a: ", a, " | r: ", r)
            s = sp
        end
        return rtotal
    end
    @test rtotaltest(m,rtotal,s,acs) == sum(rewarddist) + acs_len*running_cost - rewarddist[getind(sinit.robot)...]

    sinit = FullState([1,1], [3,3], trues(prod(mapsize)), 1000)
    m = create_target_search_pomdp(sinit, size=mapsize, rewarddist=rewarddist, maxbatt=1000, options=Dict(:observation_model=>:falco))
    s = sinit

    @test rtotaltest(m,rtotal,s,acs) == sum(rewarddist) + acs_len*running_cost - rewarddist[getind(sinit.robot)...]
end