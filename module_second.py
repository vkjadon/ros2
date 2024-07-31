import module_first

print("Imported 'module_first' So, above is the out from import command")

#Running main of imported 'module_first'
print("Below is the output of invoking main() method of First Module")
module_first.main()

def main():
    print("The name of the Second Module is {} ".format(__name__))

if __name__=='__main__':
    main()