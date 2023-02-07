using Compose
set_default_graphic_size(14cm,4cm)
rawimg = read(joinpath(@__DIR__,"..","drone.png"));
X = 0.9*rand(10,2)

img = compose(context(),
    (context(), rectangle(), fill("transparent"), stroke("orange")),
    (context(), bitmap(["image/png"], [rawimg], X[:,1], X[:,2], [0.1], [0.1]))
)