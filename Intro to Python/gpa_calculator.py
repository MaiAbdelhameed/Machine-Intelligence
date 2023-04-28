from typing import List
from college import Student, Course
import utils

def calculate_gpa(student: Student, courses: List[Course]) -> float:
    '''
    This function takes a student and a list of course
    It should compute the GPA for the student
    The GPA is the sum(hours of course * grade in course) / sum(hours of course)
    The grades come in the form: 'A+', 'A' and so on.
    But you can convert the grades to points using a static method in the course class
    To know how to use the Student and Course classes, see the file "college.py"  
    '''
    #TODO: ADD YOUR CODE HERE
    sum1=0
    sum2=0
    for course in courses:
        if student.id in course.grades.keys():
            sum1+=(course.hours*course.convert_grade_to_points(course.grades[student.id]))
            sum2+=course.hours

    if sum2==0:
        return 0
    else: 
        score=float(sum1/sum2)
        return score
    


