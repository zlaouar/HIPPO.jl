Base.@kwdef mutable struct BaselineSimulator 
    msim::TargetSearchPOMDP 
    sinit::TSState         
    rewardframes::Frames    = Frames(MIME("image/png"))
    dt::Float64             = 1/10
    max_iter::Int           = 500
    display::Bool           = false
    verbose::Bool           = true
end

function remove_rewards(pomdp, s)
    pomdp.reward[rewardinds(pomdp,s)...] = 0.0
end

function remove_rewards_sim(pomdp, rewardmdat, s)
    rewardmdat[rewardinds(pomdp,s)...] = 0.0
end

function convertinds(m::TargetSearchPOMDP, pos::Union{Vector{Int}, Tuple{Int,Int}})
    correct_ind = reverse(pos)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
    return inds
end

function ind2pos(mapsize::Tuple{Int, Int}, ind::Union{Vector{Int}, Tuple{Int,Int}})
    correct_ind = reverse(ind)
    yind = mapsize[2] - correct_ind[2] + 1
    inds = [correct_ind[1], yind]
    return inds
end

function simulateBaseline(sim::BaselineSimulator, pospoints, polypoints)
    (;msim,max_iter) = sim
    r_total = 0.0
    d = 1.0
    s = sim.sinit
    iter = 0
    history = NamedTuple[]
    posind = 1
    numpoints = length(pospoints)
    simreward = deepcopy(msim.reward)
    polyflag = true
    while !isterminal(msim, s, s.target) && iter < max_iter
        tm = time()
        a = action(s.robot, pospoints[posind])
        #remove_rewards(msim, s.robot) # remove reward at current state
        newrobot = bounce(msim, s.robot, actiondir[a])
        s = FullState(newrobot, s.target, vec(trues(size(msim.reward))), s.battery-1)
        r_total += d*simreward[rewardinds(msim, s.robot)...]
        remove_rewards_sim(msim, simreward, s.robot) # remove reward at current state
        d *= discount(msim)
        #sim.verbose && println("s: ", s.robot, " | next s: ", pospoints[posind])
        sim.verbose && println("cell ", posind, " of ", numpoints, ", robot: ", s.robot, ", next: ", pospoints[posind]," | target: ", s.target)
        rewardframe = render(msim, (sp=s,), pospoints)
        sim.display && display(rewardframe) 
        sim.display && sleep_until(tm += sim.dt)
        iter += 1
        #push!(sim.rewardframes, rewardframe)
        push!(history, (s=s, a=a))
        if newrobot == pospoints[posind]
            posind += 1
        end
        if iter == numpoints && polyflag # lawnmower over polygons
            println("polygon search...")
            pospoints = polypoints
            posind = 1
            numpoints = length(polypoints)
            polyflag = false
            fullflag = true

        end
        if iter == numpoints && fullflag # exhaustive search
            println("exhaustive search...")
            pospoints = gridpoints(msim.size)
            posind = 1
            numpoints = length(polypoints)
        end
    end

    return history, r_total
end