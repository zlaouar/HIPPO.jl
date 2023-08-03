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
            a = policy(TSStateBasic(s, env.targetInit))
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


function renderVIPolicy(policy, mdp, s)
    gw = GridWorldEnv(mdp, rewarddist, s.target, size=mapsize, robotInit=s.robot)
    vi_policy = s -> action(policy, s)
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
