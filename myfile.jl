function main(col)
	rew = 0.0
	disc = 1.0
	gamma = 0.95
	x,y = size(msolve.reward)
	rvec = msolve.reward[:,col] |> reverse
		for i in 1:20
			if i > y
				rew += disc*(rvec[end]-1)
			else 
				rew += disc*(rvec[i]-1)
			end
			disc *= gamma
			display(rew)
		end
	return rew
end


