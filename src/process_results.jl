function results(file)
    data = load(file)
    find_ratio_hippo = sum(data["hippo_targetfound_vec"]) ./ length(data["hippo_targetfound_vec"])
    find_ratio_baseline = sum(data["baseline_targetfound_vec"]) ./ length(data["baseline_targetfound_vec"])
    reward_time_hippo = mean(data["hippo_reward_vec"] ./ data["hippo_time"])
    reward_time_baseline = mean(data["baseline_reward_vec"] ./ data["baseline_time"])
    println("hippo -- rtf: ", reward_time_hippo, " |  find ratio: ", find_ratio_hippo)
    println("baseline -- rtf: ", reward_time_baseline, " |  find ratio: ", find_ratio_baseline)
end

function results_avg(files)
    datavec = []
    for file in files
        data = load(file)
        push!(datavec, data)
    end
    hippo_reward_vec = vcat(get.(datavec, "hippo_reward_vec", 1)...)
    hippo_time_vec = vcat(get.(datavec, "hippo_time", 1)...)
    hippo_targetfound_vec = vcat(get.(datavec, "hippo_targetfound_vec", 1)...)

    baseline_reward_vec = vcat(get.(datavec, "baseline_reward_vec", 1)...)
    baseline_time_vec = vcat(get.(datavec, "baseline_time", 1)...)
    baseline_targetfound_vec = vcat(get.(datavec, "baseline_targetfound_vec", 1)...)

    sims = length(hippo_reward_vec)
    println("sims per file: ", sims)
    hippo_rtf = hippo_reward_vec ./ hippo_time_vec
    hippo_rtf_mean = mean(hippo_rtf)
    hippo_rtf_std = std(hippo_rtf) / sqrt(sims)
    hippo_targetfound_ratio = sum(hippo_targetfound_vec) / length(hippo_targetfound_vec)

    baseline_rtf = baseline_reward_vec ./ baseline_time_vec
    baseline_rtf = baseline_rtf[broadcast(!, isnan.(baseline_rtf))]
    baseline_rtf_mean = mean(baseline_rtf)
    baseline_rtf_std = std(baseline_rtf) / sqrt(sims)
    baseline_targetfound_ratio = sum(baseline_targetfound_vec) / length(baseline_targetfound_vec)

    println("hippo -- rtf: ", hippo_rtf_mean, " ± ", hippo_rtf_std, " |  find ratio: ", hippo_targetfound_ratio)
    println("baseline -- rtf: ", baseline_rtf_mean, " ± ", baseline_rtf_std, " |  find ratio: ", baseline_targetfound_ratio)
end