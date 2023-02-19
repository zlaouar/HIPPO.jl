function main(col)
	rew = 0.0
	gamma = 0.95
	oggam = 0.95
	x,y = size(msolve.reward)
	rvec = msolve.reward[:,col] |> reverse
		for i in 1:20
			if i > y
				rew += gamma*(rvec[end]-1)
			else 
				rew += gamma*(rvec[i]-1)
			end
			gamma *= oggam
		end
	return rew
end


