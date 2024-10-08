using HIPPO
using POMDPs
using Test
using JLD2

#println("ARGS: ", ARGS)

#mapid = parse(Int, ARGS[1]) 
#startid = parse(Int,ARGS[2])

mapid = 1
startid = 1

filevec = ["../data/opdata1.jld2", 
           "../data/opdata2.jld2",
           "../data/opdata3.jld2",
           "../data/opdata4.jld2",
           "../data/opdata5.jld2"]

#file = filevec[mapid]
file = filevec[1]
inputs = jldopen(joinpath("data", file), "r")["inputs"]
db = jldopen(joinpath("data", file), "r")["db"]

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
rewarddist = abs.(rewarddist)
mapsize = reverse(size(rewarddist))
northstart = HIPPO.ind2pos(mapsize, db.ID2grid[40607])
southstart = HIPPO.ind2pos(mapsize, db.ID2grid[19064])
weststart = HIPPO.ind2pos(mapsize, db.ID2grid[28390])
northeaststart = HIPPO.ind2pos(mapsize, db.ID2grid[45650])

startvec = [northstart, southstart, weststart, northeaststart]

robotinit = startvec[startid]
maxbatt = 500

target = HIPPO.newtarget(mapsize, db)
newtarget_func = HIPPO.newtarget

sinit = UnifiedState(robotinit, target, vec(trues(mapsize)), maxbatt, false, :up)


cam_info = HIPPO.CameraInfo(
    deg2rad(71.5), # horizontal fov
    deg2rad(56.8), # vertical fov
)

pomdp = UnifiedPOMDP(sinit, 
                    size=mapsize, 
                    rewarddist=rewarddist, 
                    maxbatt=maxbatt,
                    camera_info=cam_info)

@testset "🦅" begin
    #All Target
    test_pomdp = UnifiedPOMDP(sinit, 
                        size=mapsize, 
                        rewarddist=rewarddist, 
                        maxbatt=maxbatt,
                        camera_info=cam_info,
                        target_bias=1.0)
    for i in 1:mapsize[1]
        for j in 1:mapsize[2]
            s = UnifiedState([i,j], [0,0], vec(trues(mapsize)), maxbatt, false, :up)
            s1 = UnifiedState([i,j], [i,j], vec(trues(mapsize)), maxbatt, true, :up)
            for a in actions(test_pomdp)
                @test reward(test_pomdp,s,a,s) == -1.0
                @test reward(test_pomdp,s1,a,s1) == 2*1000.0
            end
        end
    end
    #All Reward
    test_pomdp = UnifiedPOMDP(sinit, 
                        size=mapsize, 
                        rewarddist=rewarddist, 
                        maxbatt=maxbatt,
                        camera_info=cam_info,
                        target_bias=0.0)
    for i in 1:mapsize[1]
        for j in 1:mapsize[2]
            s = UnifiedState([i,j], [0,0], vec(trues(mapsize)), maxbatt, false, :up)
            s1 = UnifiedState([i,j], [i,j], vec(trues(mapsize)), maxbatt, true, :up)
            for a in actions(test_pomdp)
                @test reward(test_pomdp,s,a,s) == -1.0+2*rewarddist[HIPPO.rewardinds(test_pomdp, [i,j])...]
                @test reward(test_pomdp,s1,a,s1) == 2*rewarddist[HIPPO.rewardinds(test_pomdp, [i,j])...]
            end
        end
    end
    #0.5
    test_pomdp = UnifiedPOMDP(sinit, 
                        size=mapsize, 
                        rewarddist=rewarddist, 
                        maxbatt=maxbatt,
                        camera_info=cam_info,
                        target_bias=0.5)
    for i in 1:mapsize[1]
        for j in 1:mapsize[2]
            s = UnifiedState([i,j], [0,0], vec(trues(mapsize)), maxbatt, false, :up)
            s1 = UnifiedState([i,j], [i,j], vec(trues(mapsize)), maxbatt, true, :up)
            for a in actions(test_pomdp)
                @test reward(test_pomdp,s,a,s) == -1.0+rewarddist[HIPPO.rewardinds(test_pomdp, [i,j])...]
                @test reward(test_pomdp,s1,a,s1) == rewarddist[HIPPO.rewardinds(test_pomdp, [i,j])...] + 1000.0
            end
        end
    end
end