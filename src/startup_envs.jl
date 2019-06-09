#ENV["JULIA_NUM_THREADS"] = "4"
#ENV["UV_THREADPOOL_SIZE"] = "4"
#ENV["JULIA_REVISE"] = "manual" # manually call revise() to update code
#ENV["JULIA_REVISE_INCLUDE"] = "1" # automatic tracking included files
#ENV["JULIA_REVISE_POLL"] = "1"
ENV["AI_THREADS"] = "1"
julia = joinpath(Sys.BINDIR, "julia")
start = abspath(@__DIR__,"startup.jl")
run(`$julia $start`)
