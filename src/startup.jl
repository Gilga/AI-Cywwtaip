using Distributed
AI_THREADS = haskey(ENV, "AI_THREADS") ? parse(Int64, ENV["AI_THREADS"]) : 1
if AI_THREADS > 1 addprocs(AI_THREADS) end # total: procs + main proc
@info "Start $(nprocs()) processes"

@everywhere begin
  using Distributed
  @info "Start process $(myid())"

  @eval using Pkg
  @eval pkg"activate ." # activate Project
  #@everywhere Pkg.instantiate()

  # add current path
  cd(@__DIR__)
  push!(LOAD_PATH,@__DIR__)
end

################################################################################

const FILE_SCRIPT_ERROR = abspath(joinpath(@__DIR__,"../logs/error_script.log"))
const FILE_OUTPUT = abspath(joinpath(@__DIR__,"../logs/output.log"))
const FILE_REVISELOG = abspath(joinpath(@__DIR__,"../logs/revise.log"))

clearLog() = open(FILE_OUTPUT,"w+") do f write(f,"") end

################################################################################

messages=Channel{String}(Inf)

function sendToPrinter(msg::String)
  global messages
  put!(messages, msg)
end

function Printer()
  global FILE_OUTPUT, messages
  while true
    msgs = ""
    while isready(messages) msgs*=take!(messages) end
    if length(msgs)>0
      open(FILE_OUTPUT,"a+") do f write(f,msgs) end
    end
    sleep(1)
  end
end

################################################################################

logs = nothing

try
    @eval using Revise
    @eval using Base.CoreLogging: Debug

    rlogger = Revise.debug_logger()
    Revise.ReviseLogger(Revise.LogRecord[], Debug)
    logs = filter(r->r.level==Debug, rlogger.logs)
catch ex
  error("Revise Error! Could not load Revise: $ex")
end

################################################################################
# preloads
@everywhere using cywwtaip
@everywhere cywwtaip.init()

################################################################################

print("\nPress enter to start...")
input = readline() # wait

################################################################################

# Green Threads
@async Printer()
#@async Helper()

################################################################################
input = ""

while input != "q" && input != "quit"
  global logs, input

  try

    clearLog()
    println("Start Script...")
    @eval using Client
    Client.main(ARGS;printer=sendToPrinter)

  catch ex
    println("Script Error! For more details look in $FILE_SCRIPT_ERROR")
    open(FILE_SCRIPT_ERROR,"w+") do f Base.showerror(f, ex, catch_backtrace()) end
  end

  if logs != nothing open(FILE_OUTPUT, "w+") do io for log in logs println(io, log) end end end

  print("\nPress enter to restart...")
  input = readline()
  println("Reload Script...")

  try
    Revise.revise()
  catch ex
    println("Revise Error! For more details look in $FILE_REVISELOG")
    open(FILE_REVISELOG,"w+") do f Base.showerror(f, ex, catch_backtrace()) end
  end

  println("\33[2J") # clear console: https://rosettacode.org/wiki/Terminal_control/Clear_the_screen#Julia
end
