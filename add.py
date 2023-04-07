def main():
    print("Let's implement addition. Type two numbers for x and y.")

    x = int(input("x > "))
    y = int(input("y > "))
    
    print("%d + %d = %d" % (x, y, add(x, y)))    
    

# TODO: add() 함수 정의

def add(x,y):
    return x+y

if __name__ == "__main__":
    main()

