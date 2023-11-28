using HIPPO
using POMDPs
using BasicPOMCP
using POMDPTools
using DiscreteValueIteration
using Profile
using ParticleFilters
using D3Trees
using JSON, FileIO
using StaticArrays

rewarddist = [-3.08638     1.04508  -38.9812     6.39193    7.2648     5.96755     9.32665   -9.62812   -0.114036    7.38693      3.39033   -5.17863  -12.7841;
-8.50139     2.3827   -30.2106   -74.7224   -33.9783    -3.63283    -4.73628   -6.19297   -4.34958    -6.13309    -36.2926    -7.35857    0.417866;
-12.0669      7.54123  -22.8483   -47.2838   -53.8302   -25.5759    -36.2189    -4.93866   -4.9971    -12.1572     -15.8788   -23.9603   -15.3152;
-11.2335     -5.6023   -32.8484   -58.316    -35.6942   -40.4131    -80.1317     4.50919    0.302756   -0.238148     7.76839    2.78539   39.5031;
-7.1815     -5.4408   -26.9287   -61.4094   -50.8607   -36.6915    -17.6218    -7.06897    0.190177   -0.0721576    8.61714   41.2753    69.0911;
2.89205   -14.3239   -87.9894   -64.7747   -68.2573   -45.2064    -62.6445   -59.5357   -32.3136    -52.6505       7.37878   28.6342    31.5646;
-0.741237  -15.9554   -83.2767   -69.0195   -82.2122   -45.17      -21.2148    -8.11823    8.68415    16.4957       8.32323   16.4972    14.3504;
6.44794     7.12914  -88.2391   -68.5625   -74.8771   -21.2487    -11.021     -2.84843   -5.2219      1.83158     13.7386    -4.35878   17.0571;
11.0371      2.88455  -59.5524   -35.7124   -35.061     -9.27868    -8.9189    -8.82431  -51.8993      9.63887    -13.3222   -21.0979   -14.339;
7.90618     3.18679  -61.3164   -70.7954   -35.6381    -5.88295   -51.0393   -31.984    -49.4399    -25.144      -10.1865   -33.8935   -23.4304;
4.06703     9.92574   -9.96883  -48.9633   -55.4547   -29.8576    -37.7918   -49.4194   -25.8577    -34.64        15.5699     6.30979    8.75206;
6.93365     2.50252    9.63002    5.05564   -1.67295  -46.427     -69.802    -58.4468   -48.2396     -9.09721     20.3898    11.296      1.68226;
6.69843     0.88624    6.50904    7.60138  -15.8097   -55.7776    -39.8913   -56.2164     4.2347      2.45662      4.0834     4.77346    0.373309;
4.5434      1.84961    5.05996    1.71024  -16.2119   -70.8986    -68.3217   -42.1496    13.7424     14.7261       1.78606    8.92938    0.35768;
5.93137     2.38837    5.00692    2.17936   -6.58787  -48.8138    -27.0167   -10.6387     1.24938    21.9765       4.26369    6.6729     2.1039;
6.35598     1.425      2.92712    4.96801   13.0207    -0.589068  -15.8313    10.7642    16.1614     15.3144       3.59158    7.8918     9.1199]
smallreward = [800.0 2.0 2.0 -20.0;
                2.0 2.0 2.0 2.0;
                2.0 2.0 2.0 2.0;
                2.0 2.0 2.0 2.0;
                2.0 2.0 2.0 2.0;
                2.0 2.0 2.0 2.0]
hallway = [80.0 80.0;
           50.0 50.0;
           20.0 20.0;
           -100.0 -100.0;
           0.0 0.0;
           0.0 0.0;
           0.0 0.0]
#rewarddist = smallreward
#rewarddist = hallway
#rewarddist = load("rewardmat.jld2","rewarddist")
#rewarddist = rewarddist .+ abs(minimum(rewarddist)) .+ 0.01
rewarddist = abs.(rewarddist)
mapsize = reverse(size(rewarddist)) #(13,16)
maxbatt = 1000
sinit = FullState([3,1], [10,11], vec(trues(mapsize)), maxbatt)#rand(initialstate(msim))
#mapsize = (13,16)
#sinit = TSState([10,1],[13,16],trues(prod(mapsize)))#rand(initialstate(msim))
#mapsize = (4,4)
#sinit = TSState([1,1],[4,4],trues(prod(mapsize)))#rand(initialstate(msim))
sinitBasic = BasicState(sinit.robot,sinit.target)


pomdp = FullPOMDP(sinit, 
                    size=mapsize, 
                    rewarddist=rewarddist, 
                    maxbatt=maxbatt)

basic_pomdp = BasicPOMDP(sinitBasic, size=mapsize)
mdp_solver = ValueIterationSolver() # creates the solver
mdp_policy = solve(mdp_solver, UnderlyingMDP(basic_pomdp))

p = FunctionPolicy(FixedPolicy())
mdprollout = FORollout(TargetSearchMDPPolicy(mdp_policy))
funcrollout = FORollout(p)
#mdprollout = FORollout(mdp_policy) # change MDP reward mat to pompdp reward mat
#solver = POMCPSolver(estimate_value = mdprollout, tree_queries=10000, max_time=0.2, c=5) # mdp policy rollout
#solver = POMCPSolver(estimate_value = funcrollout, tree_queries=10000, max_time=0.2, c=5) # up rollout
solver = POMCPSolver(tree_queries=10_000, max_time=0.2, c=5) # random


planner = solve(solver,pomdp)


b0 = initialstate(pomdp)
N = 1000
particle_up = BootstrapFilter(pomdp, N)
particle_b = initialize_belief(particle_up, b0)


#a, info = action_info(planner, Deterministic(TSState([13,4],mapsize,vec(trues(mapsize)))), tree_in_info=true)
#inchrome(D3Tree(info[:tree], init_expand=3))

hipposim = HIPPOSimulator(msim=pomdp, planner=planner, up=particle_up, b=particle_b, sinit=sinit, dt=1/10, max_iter=maxbatt, display=true)
hist, r_total = simulateHIPPO(hipposim)

#renderVIPolicy(mdp_policy, basic_pomdp, sinitBasic) # render MDP policy


println("Total Reward: ", r_total)
