

function falcoPOMDP()
    # Transition Matrix
    T = zeros(2,3,2) # |S|x|A|x|S|, T[s', a, s] = p(s'|a,s)
    # Alert action
    T[1,1,1]=1
    T[2,1,2]=1
    # Gather info action
    T[1,2,1]=0.5
    T[2,2,1]=0.5
    T[1,2,2]=0.2
    T[2,2,2]=0.8
    # Continue
    T[1,3,1]=0.6
    T[1,3,2]=0.4
    T[2,3,1]=0.4
    T[2,3,2]=0.6

    # Observation Matrix - "Gather info: 2"
    O = zeros(2,3,2) # |O|x|A|x|S|, O[o, a, s] = p(o|a,s)
    confidence_score = 0.9
    # Confidence high
    O[1,2,1] = 1-confidence_score
    O[1,2,2] = confidence_score
    # Confidence low
    O[2,2,1] = confidence_score
    O[2,2,2] = 1-confidence_score

    for a in [1,3]
        for s in 1:2
            O[1, a, s] = 0.5 # some default value
            O[2, a, s] = 0.5 # ensure probabilities sum to 1
        end
    end


    # Reward Matrix
    R = zeros(2,3) # |S|x|A|, R[s, a]
    R[1,1]=-100
    R[1,2]=0
    R[1,3]=80
    R[2,1]=100
    R[2,2]=-75
    R[2,3]=-75

    # Model
    discount = 0.95
    return TabularPOMDP(T, R, O, discount)
end