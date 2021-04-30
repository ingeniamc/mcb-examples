def version_gen = '000'
    
@Library('summit-builder@bugfix/CITT-23-cococypher-doesnt-take-into-acco')_

proj = new project.Map()
parallelList = [:]

node('publisher-1.0')
{   
    deleteDir()
    
    stage('Repos checkouts') {
        checkout scm
    }
    
    if (env.BRANCH_NAME != 'master') {
        version_gen = VersionNumber('${BUILDS_ALL_TIME, XXX}')
    }
    
    parallelList = proj.checkout('ciConfig.yaml', version_gen)
}

parallel parallelList