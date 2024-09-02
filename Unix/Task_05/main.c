#include <stdio.h> //printf, errno
#include <fcntl.h> //open
#include <unistd.h> //read, lseek, close
#include <stdlib.h> //malloc
#include <errno.h> //errno
#include <stdbool.h> //bool
#include <string.h> //strchr
#include <poll.h> //poll

#define INC_COEF 1.5
#define DEFAULT_CAPACITY 8

typedef struct _IndentTable
{
	off_t* arr;
	size_t size;
	size_t cap;
} IndentTable;


void initIndentTable(IndentTable* table)
{
	errno = 0;

	table->arr = calloc(DEFAULT_CAPACITY, sizeof(off_t));
	if(table->arr == NULL)
	{
		perror("Cannot allocate memory for ident array\n");
		return;
	}

	table->size = 0;
	table->cap = DEFAULT_CAPACITY;
}

bool resizeTable(IndentTable* table)
{
	errno = 0;

	off_t* oldArr = table->arr;
	size_t oldCap = table->cap;

	size_t newCap = oldCap * INC_COEF;

	table->arr = realloc(table->arr, newCap * sizeof(off_t));
	if (table->arr == NULL)
	{
		table->arr = oldArr;
		perror("Cannot realloc memory\n");
		return false;
	}
	table->cap = newCap;

	return true;
}

bool pushIndent(IndentTable* table, off_t val)
{
	if(table->size == table->cap)
	{
		if(!resizeTable(table))
		{
			return false;
		}
	}

	table->arr[table->size] = val;
	++table->size;
	return true;
}

#define BUFF_SIZE 16
#define TIMEOUT 1000

bool waitRead(int fildes, int timeout)
{
	struct pollfd fds = {
			.fd = fildes,
			.events = POLLIN,
			.revents = 0
	};

	return poll(&fds, 1, timeout) == 1;
}

bool fillIndentTable(IndentTable* table, int fildes)
{
	errno = 0;

	char buff[BUFF_SIZE + 1] = {0};
	off_t currPos = 0;
	ssize_t readCount = 0;

	int oldFl = fcntl(fildes, F_GETFL);

	while((readCount = read(fildes, buff, BUFF_SIZE)) != 0)
	{
		if(readCount == -1)
		{
			if (errno == EINTR)
			{
				errno = 0;
				perror("Signal caught in read\n");
				continue;
			}

			if (errno == EAGAIN)
			{
				if (waitRead(fildes, TIMEOUT))
				{
					errno = 0;
					continue;
				}
				else
				{
					perror("No data waiting to be read\n");
					break;
				}
			}

			perror("Error in reading file\n");
			fcntl(fildes, F_SETFL, oldFl);
			return false;
		}

		buff[readCount] = '\0';
		char* endlinePos = buff;

		while((endlinePos = strchr(endlinePos, '\n')) != NULL)
		{
			if(!pushIndent(table, currPos + (endlinePos - buff)))
			{
				fcntl(fildes, F_SETFL, oldFl);
				return false;
			}
			++endlinePos;
		}

		currPos += readCount;
	}

	fcntl(fildes, F_SETFL, oldFl);
	return true;
}

void printIndentTable(IndentTable* table)
{
	printf("{ ");
	for(size_t i = 0; i < table->size; ++i)
	{
		printf("%lld, ", table->arr[i]);
	}
	printf(" }\n");
}

void destroyIndentTable(IndentTable* table)
{
	if(table->arr != NULL)
	{
		free(table->arr);
	}
}

bool printLine(int fildes, IndentTable* table, int lineNum)
{
	errno = 0;

	--lineNum;

	if(lineNum < 0 || lineNum >= table->size)
	{
		printf("No line with such number\n");
		return false;
	}

	off_t beginPos =  lineNum == 0
		  	? 0
			: table->arr[lineNum-1] + 1; //+1 to get next pos after '\n'
	off_t length = table->arr[lineNum] - beginPos  + 1; //+1 to include '\n'

	char* line = calloc(length + 1, sizeof(char));
	if(line == NULL)
	{
		perror("Cannot allocate memory to printLine buffer\n");
		return false;
	}


	lseek(fildes, beginPos, SEEK_SET);
	read(fildes, line, length);

	printf("%s", line);
	free(line);

	return true;
}

int main(int argc, char* argv[])
{
    if(argc < 2){
        perror("Not enough arguments\n"
               "Example: ./a.out test.txt\n");
        return -1;
    }

	errno = 0;

	int fildes = open(argv[1], O_RDONLY);

	if(fildes == -1)
	{
		perror("Cannot open file\n");
		return -1;
	}


	IndentTable table;
	initIndentTable(&table);

	fillIndentTable(&table, fildes);

	off_t lineNum;
	while(1)
	{
		printf("Type line num to read. Type 0 to exit: ");
		scanf("%lld", &lineNum);

		if(lineNum == 0)
		{
			break;
		}

		printLine(fildes, &table, lineNum);
	}

	destroyIndentTable(&table);

    while(1)
    {
        if(close(fildes) == -1)
        {
            if (errno == EINTR)
            {
                perror("Signal caught\n");
                errno = 0;
                continue;
            }

            perror("Close fail\n");
            break;
        } else
            break;
    }

	return 0;
}
