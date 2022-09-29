## Git Subtree
This repository uses git subtree to manage the sub repositories.   

### Adding a sub-repository
To add a sub-repository located at sub_repo_url in the folder src/my_folder (my_folder should not exist), enter the following commands:     
```
git remote add sub_repo_name sub_repo_url    
git subtree add --prefix=src/my_folder sub_repo_name sub_repo_branch
```

To make it easier to know the name of the sub-repository, the sub-repository name is used as the folder name.   

### Pulling change from sub-repository
```
git subtree pull --prefix=src/my_folder sub_repo_name sub_repo_branch
```

### Pushing change to sub-repository
```
git subtree push --prefix=src/my_folder sub_repo_name sub_repo_branch
```
