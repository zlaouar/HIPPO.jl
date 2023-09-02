function action(curr_pos, next_pos)
    if curr_pos[1] > next_pos[1]
        return :left
    elseif curr_pos[1] < next_pos[1]
        return :right
    elseif curr_pos[2] > next_pos[2]
        return :down
    elseif curr_pos[2] < next_pos[2]
        return :up
    else 
        return :stay
    end
end 
