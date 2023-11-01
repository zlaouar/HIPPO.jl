using HIPPO
using POMDPs
using POMDPTools
using JLD2

db = load(joinpath(@__DIR__, "../data/db.jld2"), "db")
inputs1 = load(joinpath(@__DIR__, "../data/opdata1.jld2"), "inputs")
inputs2 = load(joinpath(@__DIR__, "../data/opdata2.jld2"), "inputs")
inputs3 = load(joinpath(@__DIR__, "../data/opdata3.jld2"), "inputs")
inputs4 = load(joinpath(@__DIR__, "../data/opdata4.jld2"), "inputs")
inputs5 = load(joinpath(@__DIR__, "../data/opdata5.jld2"), "inputs")

rewarddist = db.reward
rewarddist = abs.(rewarddist)
mapsize = reverse(size(rewarddist))
northstart = HIPPO.ind2pos(mapsize, db.ID2grid[40607])
southstart = HIPPO.ind2pos(mapsize, db.ID2grid[19064])
weststart = HIPPO.ind2pos(mapsize, db.ID2grid[28390])
northeaststart = HIPPO.ind2pos(mapsize, db.ID2grid[45650])

robotinit = northstart
maxbatt = 1000

opdata = inputs1
pospoints = HIPPO.getdata(opdata, db, mapsize)[end-10:end]
polypoints = HIPPO.polypoints(opdata, db, mapsize)[1:2]
sortedpoints = HIPPO.gendists(pospoints, robotinit)

sinit = FullState(robotinit, HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()]), vec(trues(mapsize)), maxbatt) #rand(initialstate(msim))


pomdp = FullPOMDP(sinit,
                  size=mapsize,
                  rewarddist=rewarddist,
                  maxbatt=maxbatt)

bsim = BaselineSimulator(msim=pomdp, sinit=sinit, dt=1/4, max_iter=maxbatt, display=true)
histvec = []
rvec = []
targetfound = Bool[]
newtarget = HIPPO.newtarget(mapsize, db)
for i in 1:2
    hist, rtot = simulateBaseline(bsim, sortedpoints, polypoints)
    newtarget = HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()])
    pomdp = FullPOMDP(sinit,
                  size=mapsize,
                  rewarddist=rewarddist,
                  maxbatt=maxbatt)
    bsim.msim = pomdp
    bsim.sinit = FullState(sinit.robot, newtarget, sinit.visited, sinit.battery)
    push!(targetfound, last(hist).s.robot == newtarget)
    push!(histvec, hist)
    push!(rvec, rtot)
end

#hist = simulateBaseline(bsim, sortedpoints)

