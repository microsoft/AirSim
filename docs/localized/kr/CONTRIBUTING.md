# 기여

## 빠른 시작
- [짧고 친절한 코딩 지침](docs/coding_guidelines.md)을 읽으십시오.
- 새로운 기능 추가 또는 리팩토링과 같은 큰 변화는 [이슈를 먼저 제기하십시오](https://github.com/Microsoft/AirSim/issues).
- [권장 개발 워크 플로우](docs/dev_workflow.md)를 사용하여 변경하고 테스트하십시오.
- 다른 GitHub 프로젝트와 마찬가지로 [일반적인 단계](https://akrabat.com/the-beginners-guide-to-contributing-to-a-github-project/)를 사용하여 기여하십시오. Git Branch-Rebase-Merge 워크 플로우에 익숙하지 않다면, [이것을 먼저 읽으십시오](http://shitalshah.com/p/git-workflow-branch-rebase-squash-merge/).

## 체크 리스트
- 당신이 선호하지 않더라도 나머지 코드와 동일한 스타일과 형식을 사용하십시오.
- 변경한 코드에 연관된 설명서(documentation)를 변경하십시오.
- OS 특유의 헤더 파일 또는 새로운 3rd party 종속성을 포함하지 마십시오.
- 가능하면 10개 이하의 적은 파일로 Pull request를 하십시오.
- 큰 바이너리 파일을 포함하지 않아야 합니다.
- 새 라이브러리나 코드(includes)를 추가 할 때는 종속성이 반드시 필요합니다.
- 마스터로 브랜치를 자주 리베이스하십시오 (2-3 일마다 한 번이 이상적입니다).
- 코드가 Windows, Linux 및 OSX에서 컴파일되는지 확인하십시오.
