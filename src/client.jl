"""
TESTIFY
"""
module Client

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

    @info "Loop..."
    while isAlive(client)
      getBotSpeed(client, 0)
      getScore(client, getMyPlayerNumber(client))
      changeMoveDirection(client, 1, -0.08f0)

      position = getBotPosition(client, 0, 0)
      direction = getBotDirection(client, 0)

      graph = getGraph(client)[1]
      for node in graph.neighbors
        println(typeof(node))
        n = GraphNode(node)
        println(toString(n) * ": " * string(n.owner) * ", " * string(n.blocked))
      end
    end
    
  catch ex
    outputBacktrace(ex)
    error("NetworkClient error! For more details look in $(getErrorLogFile())")
  end

end

end # Client