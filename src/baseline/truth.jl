# This document contains the inputted data from Amalfitano's review of the scenario and his respective truth data.
# The data contains two main sections: trail areas and locations.
# 

# 80% probability of area in locations below
eastRanchTrail = [
    16917, 16918, 16919, 16920, 17632, 17633, 17634, 17635, 17638, 17639, 17640, 17641, 18349, 18350, 18359, 19065, 
    19066, 19067, 19077, 19078, 19079, 19080, 19797, 19798, 19799, 20515, 20516, 20517, 21234, 21235, 21952, 21953, 
    22671, 22672, 23390, 23391, 24109, 24110, 24828, 25546, 25547, 26265, 26266, 26267, 26985, 26986, 26987, 27705, 
    28423, 28424, 28425, 32015, 32016, 32733, 32734, 33450, 33451, 34167, 34168, 34885, 34886, 35603, 35604, 36320, 
    36321, 37038, 37039, 37756, 38473, 38474, 39190, 39191, 39192, 39909, 39910, 41338, 41339]

location1 = [
    16196,16197,16198,16914,16915,16916]

location2 = [
    29142,29143,29144,29145,29146,29860,29861,29862,29863,29864,30578,30579,30580,30581,30582,31296,31297,31298,31299,
    31300]

location5 = [
    40626,40627,40628,41340,41341,41342,41343,41344,41345,41346,42057,42058,42059,42060,42061,42062,42063,42064,42775,
    42776,42777,42778,42779,42780,42781,42782,43493,43494,43495,43496,43497,43498,43499,43500,44212,44213,44214,44215,
    44216,44217,44218]

# 20% probability of area in locations below
westRanchTrail = [
    19783, 19784, 20502, 21220, 21938, 22655, 22656, 23372, 23373, 24089, 24090, 24807, 24808, 25526, 25527, 26245, 
    26246, 26964, 27682, 28399, 28400, 29115, 29833, 30551, 30552, 31270, 31271, 31989, 33426, 33427, 33428, 34144, 
    34145, 34146, 34863, 35581, 36298, 36299, 36302, 36303, 36304, 37016, 37019, 37020, 37022, 37023, 37733, 37734, 
    37737, 37741, 37742, 38448, 38449, 38450, 38451, 38454, 38455, 38460, 38461, 38462, 38463, 38464, 38465, 39167, 
    39168, 39169, 39171, 39172, 39180, 39182, 39887, 39888, 39889, 39900, 39901, 40606, 40607, 40619, 40620, 41338]

location3 = [
    26242,26243,26244,26959,26960,26961,26962,27677,27678,27679,27680,28396,28397,28398]

location4 = [
    31272,31273,31274,31275,31990,31991,31992,31993,32708,32709,32710,32711]

location6 = [
    21252, 21253, 21970, 22687, 22688, 23404, 23405, 24118, 24119, 24120, 24121, 24122, 24836, 25554, 26272, 26273, 
    26274, 26992, 27709, 27710, 28427]

area80POA = vcat(eastRanchTrail, location1, location2, location5)
area20POA = vcat(westRanchTrail, location3, location4, location6)

function chooseTrueCell()
    # Function chooses a random cell for a person to be in based on Amalfitano's inputs
    # Returns the cell number

    # weighted random sampling
    weight80 = ones(length(area80POA))*(0.8/length(area80POA))
    weight20 = ones(length(area20POA))*(0.2/length(area20POA))
    weights = vcat(weight80, weight20)
    sampleSet = vcat(area80POA, area20POA)
    return sample(sampleSet, Weights(weights))
end

newtarget(mapsize, db) = HIPPO.ind2pos(mapsize, db.ID2grid[chooseTrueCell()])