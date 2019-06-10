"""
TESTIFY
"""
module Client

#using Dates

using cywwtaip  # HTW lib from AI professor

####################################################################################################

ROOT = joinpath(@__DIR__,"../")
output=Base.println
pid = 0 # process id
playerNumber = 1
name = nothing
winMsg  = nothing
server = nothing
client = nothing
initalized = false
latency = 0

RuntimeException(msg::String) = ErrorException(msg)
setOutput(poutput) = global output = poutput

getErrorLogFile() = joinpath(ROOT, "error_$(name).log")
getLogFile() = joinpath(ROOT, "out_$(name).log")

outputBacktrace(ex) = open(getErrorLogFile(),"w+") do f Base.showerror(f, ex, catch_backtrace()) end

####################################################################################################

function main(args::Array{String,1} ;output=Base.println)
  println("--------------------------------------------------------------------------------")
  @info "Start Game..."

  global playerNumber, latency, initalized

  len = length(args)
  if len == 0 throw(RuntimeException("No PID set!")) end # no pid? -> leave

  initalized = false

  #setOutput(output)

  global pid = length(args[1])>0 ? parse(Int,args[1]) : 0
  global server = len>1 ? args[2] : "localhost"
  global name = len>2 ? args[3] : string("Client",pid)
  global winMsg = len>3 ? args[4] : "SUPER!"
  global client

  try
    cywwtaip.init()
    @info "Connect..."

    # Problem: if connection gets stuck somehow the program has to be restarted...
    # There is currently no function to interrupt a task in julia...
    client = NetworkClient(server, name, winMsg)
    global playerNumber = getMyPlayerNumber(client)

    start = false
    prev_position = nothing
    startTime = time()
    tickTime = 0
    timeLimit = 60 # 60s
    timeLimitBeforeStart = 10 # 60s

    @info "Loop..."
    while isAlive(client)
      currentTime = (time() - startTime)

      if !start
        position = [
        getBotPosition(client, playerNumber, 0),
        getBotPosition(client, playerNumber, 1),
        getBotPosition(client, playerNumber, 2)]
        if prev_position != nothing && prev_position != position start=true end
        prev_position = position

        if currentTime > timeLimitBeforeStart break end # auto close

        if (time() - tickTime) > 1
          println("Remaining Time($timeLimitBeforeStart): $(timeLimitBeforeStart - round(currentTime))")
          tickTime=time()
        end

        if !start continue end

        println("\n[START]\n------------------------------")
        tickTime=0
        startTime = time()-1
        currentTime = (time() - startTime)
      end

      if (time() - tickTime) > 1
        println("Remaining Time($timeLimit): $(timeLimit - round(currentTime))")
        tickTime=time()
      end

      if currentTime > timeLimit break end #after Xs break this loop

      speed = [getBotSpeed(client, 0),getBotSpeed(client, 1),getBotSpeed(client, 2)]
      score = getScore(client, playerNumber)
      position = getBotPosition(client, 0, 0)
      direction = getBotDirection(client, 0)

      changeMoveDirection(client, 1, -0.08f0)

      #println("Speed: $speed")
      #println("Score: $score")
      #println("Position: $position")
      #println("Direction: $direction")

      #=
      graph = getGraph(client)[1]
      for node in graph.neighbors
        println(typeof(node))
        n = GraphNode(node)
        println(toString(n) * ": " * string(n.owner) * ", " * string(n.blocked))
      end
      =#
    end

  catch ex
    outputBacktrace(ex)
    error("NetworkClient error! For more details look in $(getErrorLogFile())")
  end

  print("Game ended.")
  cywwtaip.destroy() # cleanup cywwtaip
end

end # Client
