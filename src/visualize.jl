mutable struct GridWorldEnv
    m::BasicPOMDP
    size::SVector{2, Int}
    rewards::Matrix{Float64}
    robotInit::SVector{2, Int}
    targetInit::SVector{2, Int}
end

function GridWorldEnv(m, rewards::Matrix{Float64}, targetInit; size=(10,10), robotInit=(1,1))
    return GridWorldEnv(m, SA[size[1], size[2]], rewards, robotInit, targetInit)
end


"""
    render(env::GridWorldEnv)
    render(env::GridWorldEnv, color=s->5.0, policy=s->SA[1,0])

Render a GridWorldEnv to a Compose.jl object that can be displayed in a Jupyter notebook or ElectronDisplay window.

# Keyword Arguments
- `color::Function`: A function that determines the color of each cell. Input is a state, output is either a Float64 between -10 and 10 that will produce a color ranging from red to green, or any color from Colors.jl.
- `policy::Function`: A function that allows showing an arrow in each cell to indicate the policy. Input is a state; output is an action.
"""
observations(env::GridWorldEnv) = [SA[x, y] for x in 1:env.size[1], y in 1:env.size[2]]

function renderMDP(env::GridWorldEnv; color::Function=s->get(env.rewards, s, -0.1), policy::Union{Function,Nothing}=nothing)
    nx, ny = env.size
    m = env.m
    cells = []
    for s in observations(env)
        r = env.rewards[rewardinds(m, SA[s...])...]
        clr = get(ColorSchemes.redgreensplit, (r+10.0)/20.0)
        cell = context((s[1]-1)/nx, (ny-s[2])/ny, 1/nx, 1/ny)
        if policy !== nothing
            a = policy(BasicState(s, env.targetInit))
            txt = compose(context(), Compose.text(0.5, 0.5, aarrow[a], hcenter, vcenter), stroke("black"))
            compose!(cell, txt)
        end
        clr = tocolor(r)
        compose!(cell, rectangle(), fill(clr), stroke("gray"))
        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.5mm), cells...)
    outline = compose(context(), linewidth(1mm), rectangle(), stroke("gray"))

    s = env.robotInit
    agent_ctx = context((s[1]-1)/nx, (ny-s[2])/ny, 1/nx, 1/ny)
    agent = compose(agent_ctx, circle(0.5, 0.5, 0.4), fill("orange"))

    sz = min(w,h)
    return compose(context((w-sz)/2, (h-sz)/2, sz, sz), agent, grid, outline)
end


tocolor(x) = x
function tocolor(r::Float64)
    minr = -20.0
    maxr = 100.0
    frac = (r-minr)/(maxr-minr)
    return get(ColorSchemes.redgreensplit, frac)
end

const aarrow = Dict(:up=>'↑', :left=>'←', :down=>'↓', :right=>'→', :stay=>'⊙')


function renderVIPolicy(policy, mdp, s, rewarddist)
    gw = GridWorldEnv(mdp, rewarddist, s.target, size=mdp.size, robotInit=s.robot)
    vi_policy = s -> DiscreteValueIteration.action(policy, s)
    display(HIPPO.renderMDP(gw, policy = vi_policy))
end

function rendhist(hist, m; delay=0.1)
    for h ∈ hist
        remove_rewards(m, h.s.robot)
        display(render(m, h, true))
        sleep(delay)
    end
end 

function rendhist(hist, m, rewarddist; delay=0.1)
    m.reward = rewarddist
    for h ∈ hist
        remove_rewards(m, h.sp.robot)
        display(render(m, h, true))
        sleep(delay)
    end
end 

function POMDPTools.ModelTools.render(m::TargetSearchPOMDP, goal, 
                                            hippo::Vector{StaticArraysCore.SVector{2, Int64}}, 
                                            baseline::Vector{StaticArraysCore.SVector{2, Int64}})
    nx, ny = m.size
    cells = []
    iter = 1
    baseiter = 1
    minr = minimum(m.reward)-1
    maxr = maximum(m.reward)
    opvecbase = collect(1:length(baseline))./length(baseline)
    opbase = Dict(baseline.=>opvecbase)
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        r = m.reward[rewardinds(m, SA[x,y])...]
        if iszero(r)
            target = compose(context(), rectangle(), fill("black"), stroke("gray"))
        else
            frac = (r-minr)/(maxr-minr)
            clr = get(ColorSchemes.turbo, frac)
            target = compose(context(), rectangle(), fill(clr), stroke("gray"), fillopacity(0.3))
            #target = compose(context(), rectangle(), fillopacity(normie(m.reward[rewardinds(m,SA[x,y])...],m.reward)), fill("green"), stroke("gray"))
        end
        if [x,y] ∈ hippo || [x,y] ∈ baseline
            # if [x,y] ∈ hippo
            #     hippoclr = get(ColorSchemes.linear_kbc_5_95_c73_n256, ophippo[[x,y]])
            #     #hippocell = compose(context(), circle(), fillopacity(clamp(ophippo[[x,y]], 0.3, 1.0)), fill("red"), stroke("black"), linewidth(0.5mm))
            #     hippocell = compose(context(), circle(), fill(hippoclr), stroke("black"), linewidth(0.5mm))
            #     compose!(cell, hippocell)
            #     hippoiter += 1
            # end
            if [x,y] ∈ baseline
                baseclr = get(ColorSchemes.linear_kbc_5_95_c73_n256, opbase[[x,y]])
                basecell = compose(context(), circle(), fill(baseclr), stroke("black"), linewidth(0.5mm))            
                #basecell = compose(context(), circle(), fillopacity(clamp(opbase[[x,y]], 0.3, 1.0)), fill("black"), stroke("black"), linewidth(0.5mm))
                compose!(cell, basecell)
                baseiter += 1
            end
        else
            compose!(cell, target)
        end
        compose!(cell, target)

        

        push!(cells, cell)
        iter += 1
    end
    # hippoline = compose(context(), line(Tuple.(hippo)), linewidth(1mm), stroke("red"))
    grid = compose(context(), linewidth(0.00000001mm), cells...)
    outline = compose(context(), linewidth(0.01mm), rectangle(), fill("white"), stroke("black"))


    robot_ctx = cell_ctx(hippo[end], m.size)
    robot = compose(robot_ctx, circle(0.5, 0.5, 0.5), fill("black"))
    target_ctx = cell_ctx(goal, m.size)
    target = compose(target_ctx, star(0.5,0.5,1.8,5,0.5), fill("orange"), stroke("black"))

    #mapcells_hippo = [coord(cell, m.size) for cell ∈ hippo]
    #mapcells_base = [coord(cell, m.size) for cell ∈ baseline]
    #hippotrajec = compose(context(), line(mapcells_hippo), strokeopacity(0.6), linewidth(1mm), stroke("black"))
    #basetrajec = compose(context(), line(mapcells_base), strokeopacity(0.6), linewidth(1mm), stroke("red"))

    
    legend = compose(rect_ctx([m.size[1]-10, 5], m.size, 10, 5), rectangle(), fill("white"), stroke("black"))


    sz = min(w,h)
    return compose(context((w-sz)/2, (h-sz)/2, sz, (44/59)*sz), robot, target, grid, outline)
    #return compose(context((w-sz)/2, (h-sz)/2, sz, (44/59)*sz), legend, hippotrajec, basetrajec, robot, target, grid, outline)
end