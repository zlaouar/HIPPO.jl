using Serialization
using DataFrames
struct map_database
    ID2grid :: Dict{Int64, Tuple{Int64, Int64}}
    grid2ID::Dict{Tuple{Int64, Int64}, Int64}
    reward::Array{Float64}
    features::DataFrame
end
data = deserialize(joinpath(@__DIR__, "operator_database_NCAR_watertrail.dat"))
myfig = heatmap(reverse(data.reward, dims=1))

savefig(myfig,"rewardmap.png")