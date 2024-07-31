def main():
    print("This is the main() function of First Module and the name of the First Module is {} ".format(__name__))
    
if __name__=='__main__':
    main() 
else:
    print("This is outside the main() function of First Module")