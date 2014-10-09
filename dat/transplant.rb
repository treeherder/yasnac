# this merges numbers from PARAM.DAT with text descriptions
# of parameter meanings in params-important.txt and outputs
# the merged human-readable information to params-edited.txt

file = open 'PARAM.DAT'
paramPrefix = ''
dictionary = {}

file.each do | line |
  next if line.start_with? '/PRM'
  if line.start_with? '//'
    paramPrefix = line[2,10].chomp.chomp("PRM")
    print paramPrefix+"\n"
    dictionary[paramPrefix] = []
    next
  end
  dictionary[paramPrefix] += line.chomp.split(",")
end

print dictionary.inspect

inputfile = open 'params-important.txt'
outputfile = open 'params-edited.txt', 'w'

inputfile.each do | line |
  outputfile.write line
  if dictionary.include?(line[0,2])
    outputfile.write dictionary[line[0,2]][line[2,3].to_i]
    outputfile.write "\n" 
  end
end

outputfile.write "finished input file\n"
outputfile.close
