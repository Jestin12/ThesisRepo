text = "apple,banana,orange,grape"
fruits = [i[3:] for i in text.split(",")]
print(fruits)  # ['apple', 'banana', 'orange', 'grape']